"""Rerun live plots: a Hub consumer that batches samples into Rerun time series.

Entity layout is /<arm>/<field>/<joint>, so each plot shows all joints of one quantity.
The x axis is the `monotonic` timeline: seconds since the run's started_monotonic_ns, the same
clock as the parquet t_start_ns. Values are converted to display units here (%, V, mA); the
parquet log stays in raw register units. V/°C are plotted raw, glitches included (diagnostic view).

Sinks: optionally a spawned viewer (gRPC) and always an .rrd file in the run directory, so
every live session can be replayed with `rerun <run_dir>/telemetry.rrd`.
"""

import time
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from so101.telemetry.hub import Subscription

VIEWER_SETTLE_S = 2.0

# field -> (JointState attribute, scale to display unit, plot title)
FIELDS = {
    "position": ("position", 1.0, "position [ticks]"),
    "speed": ("speed", 1.0, "speed [ticks/s]"),
    "load": ("load", 0.1, "load [%]"),
    "current": ("current", 6.5, "current [mA]"),
    "temperature": ("temperature_c", 1.0, "temperature [°C] (raw)"),
    "voltage": ("voltage_dv", 0.1, "voltage [V] (raw)"),
}


def blueprint(arms: list[str]) -> rrb.Blueprint:
    tabs = [
        rrb.Grid(*[rrb.TimeSeriesView(origin=f"/{arm}/{field}", name=title) for field, (_, _, title) in FIELDS.items()],
                 name=arm)  # fmt: skip
        for arm in arms
    ]
    return rrb.Blueprint(rrb.Tabs(*tabs), collapse_panels=True)


class RerunView:
    def __init__(self, sub: Subscription, arms: list[str], rrd_path: Path, recording_id: str, t0_ns: int,
                 spawn: bool = True):  # fmt: skip
        self.sub, self.t0_ns = sub, t0_ns
        self.rec = rr.RecordingStream("so101", recording_id=recording_id)
        sinks = [rr.FileSink(str(rrd_path))]
        if spawn:
            rr.spawn(connect=False, recording=self.rec)  # start the viewer; set_sinks connects below
            sinks.insert(0, rr.GrpcSink())
        rr.set_sinks(*sinks, recording=self.rec)
        # send_blueprint, not set_sinks(default_blueprint=...): a viewer left open from an earlier run keeps
        # that run's active layout (e.g. /leader/... plots), so a follower-only run showed nothing (step 3f).
        rr.send_blueprint(blueprint(arms), make_active=True, recording=self.rec)
        if spawn:
            # A freshly launched viewer stalls other threads for ~0.5 s around 1 s after start (step 3e:
            # reads up to 33 ms, skipped poll slots). Let it settle before anything time-critical starts.
            time.sleep(VIEWER_SETTLE_S)
        self.samples = 0
        self.close_error: str | None = None

    def update(self) -> None:
        """Drain pending samples and send them as one column batch per entity."""
        batch = self.sub.drain()
        if not batch:
            return
        series: dict[str, tuple[list[int], list[float]]] = {}
        for sample in batch:
            snap = sample.snapshot
            t = snap.t_start_ns - self.t0_ns
            for joint, state in snap.joints.items():
                if state is None:
                    continue
                for field, (attr, scale, _) in FIELDS.items():
                    times, values = series.setdefault(f"{sample.arm}/{field}/{joint}", ([], []))
                    times.append(t)
                    values.append(getattr(state, attr) * scale)
        for path, (times, values) in series.items():
            rr.send_columns(
                path,
                indexes=[rr.TimeColumn("monotonic", duration=np.array(times, dtype="timedelta64[ns]"))],
                columns=rr.Scalars.columns(scalars=values),
                recording=self.rec,
            )
        self.samples += len(batch)

    def close(self) -> None:
        """Best effort: a closed viewer (or Ctrl-C reaching it) must not break shutdown of the run.
        The .rrd file sink still gets everything sent before the viewer went away."""
        try:
            self.update()
            self.rec.flush()
        except RuntimeError as e:  # e.g. "gRPC connection severed" (step 3f)
            self.close_error = str(e)
        finally:
            rr.disconnect(recording=self.rec)
