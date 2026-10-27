"""Always-on parquet logger: a Hub consumer that writes telemetry into a run directory.

Long format: one row per (sample, joint), so analysis can group by arm/joint without knowing
the joint count. Every `flush_s` the buffer is written as a *complete* part file
(telemetry/part-00000.parquet, ...), so a crash loses at most one flush interval and every
file on disk is readable. Read the whole run with `pyarrow.parquet.read_table(run_dir / "telemetry")`.
"""

import threading
import time

import pyarrow as pa
import pyarrow.parquet as pq

from so101.runlog.run import Run
from so101.telemetry.hub import Sample, Subscription

SCHEMA = pa.schema([
    ("arm", pa.string()),
    ("seq", pa.int64()),
    ("t_start_ns", pa.int64()),  # monotonic, snapshot start
    ("t_end_ns", pa.int64()),  # monotonic, snapshot end
    ("joint", pa.string()),
    ("position", pa.int32()),  # ticks
    ("speed", pa.int32()),  # ticks/s
    ("load", pa.int32()),  # 0.1 %
    ("voltage_dv", pa.int16()),  # 0.1 V
    ("temperature_c", pa.int16()),
    ("status", pa.int16()),
    ("moving", pa.int8()),
    ("current", pa.int32()),  # 6.5 mA units
    ("error", pa.string()),  # BusError text when the read failed (other columns null)
    ("goal", pa.int32()),  # Goal_Position commanded this cycle (SafeArm runs), else null
    ("error_byte", pa.int16()),  # status-packet error byte of this read (voltage/angle/overheat/overcurrent/overload)
])  # fmt: skip
STATE_FIELDS = ["position", "speed", "load", "voltage_dv", "temperature_c", "status", "moving", "current", "error_byte"]


def sample_rows(sample: Sample) -> list[dict]:
    snap = sample.snapshot
    rows = []
    for joint, state in snap.joints.items():
        row = {"arm": sample.arm, "seq": sample.seq, "t_start_ns": snap.t_start_ns, "t_end_ns": snap.t_end_ns,
               "joint": joint, "error": snap.errors.get(joint),
               "goal": sample.goals.get(joint) if sample.goals else None}  # fmt: skip
        for f in STATE_FIELDS:
            row[f] = None if state is None else getattr(state, f)
        rows.append(row)
    return rows


class ParquetLogger:
    def __init__(self, sub: Subscription, run: Run, flush_s: float = 5.0):
        self.sub, self.run, self.flush_s = sub, run, flush_s
        self.dir = run.dir / "telemetry"
        self.dir.mkdir(exist_ok=False)
        self.samples = 0
        self.rows = 0
        self.parts = 0
        self.error: BaseException | None = None
        self._buf: list[dict] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="parquet-logger", daemon=True)

    def start(self) -> "ParquetLogger":
        self._thread.start()
        return self

    def stop(self, timeout: float = 10.0) -> dict:
        """Drain what's left, write the final part and summary.json. Returns the summary."""
        self._stop.set()
        self._thread.join(timeout)
        summary = {
            "samples": self.samples,
            "rows": self.rows,
            "parts": self.parts,
            "dropped_by_hub": self.sub.dropped,  # samples lost before reaching the logger
            "error": repr(self.error) if self.error else None,
        }
        self.run.write_json("telemetry_summary.json", summary)
        return summary

    def __enter__(self) -> "ParquetLogger":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def _take(self, samples: list[Sample]) -> None:
        for s in samples:
            self._buf.extend(sample_rows(s))
        self.samples += len(samples)

    def _flush(self) -> None:
        if not self._buf:
            return
        table = pa.Table.from_pylist(self._buf, schema=SCHEMA)
        pq.write_table(table, self.run.path(f"telemetry/part-{self.parts:05d}.parquet"))
        self.rows += len(self._buf)
        self.parts += 1
        self._buf = []

    def _run(self) -> None:
        try:
            next_flush = time.monotonic() + self.flush_s
            while not self._stop.is_set():
                first = self.sub.get(timeout=0.2)
                if first is not None:
                    self._take([first, *self.sub.drain()])
                if time.monotonic() >= next_flush:
                    self._flush()
                    next_flush += self.flush_s
            self._take(self.sub.drain())
            self._flush()
        except BaseException as e:  # surfaced in the summary and by the caller
            self.error = e
