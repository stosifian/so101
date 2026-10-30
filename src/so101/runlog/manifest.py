"""Run manifest (step 10d; Phase 2 obligation): one record per visited pose.

Each record: what was commanded, an Arm snapshot just before and just after the "measurement moment",
the settle result, and an OPTIONAL camera block (None in Phase 1). Phase 2's capture loop fills `camera`
(image path, capture time, exposure, ...) and keeps everything else, so it extends this format.
Written as JSON lines while the run goes (crash-safe), one file per run: data/raw/<run_id>/manifest.jsonl.
"""

import json
import time
from pathlib import Path

from so101.hw.arm import ArmSnapshot
from so101.runlog.run import Run

SCHEMA_VERSION = 1


def snapshot_dict(s: ArmSnapshot | None) -> dict | None:
    if s is None:
        return None
    return {"t_start_ns": s.t_start_ns, "t_end_ns": s.t_end_ns, "errors": s.errors,
            "joints": {j: None if st is None else {"position": st.position, "load": st.load, "current": st.current,
                                                    "temperature_c": st.temperature_c, "voltage_dv": st.voltage_dv,
                                                    "moving": st.moving, "status": st.status}
                       for j, st in s.joints.items()}}  # fmt: skip


class Manifest:
    def __init__(self, run: Run, kind: str):
        self.path = run.path("manifest.jsonl")
        self._f = self.path.open("x")  # created once; appended during this run only
        self.count = 0
        self._write({"type": "header", "schema_version": SCHEMA_VERSION, "run_id": run.run_id, "kind": kind,
                     "created_monotonic_ns": time.monotonic_ns()})  # fmt: skip

    def _write(self, obj: dict) -> None:
        self._f.write(json.dumps(obj) + "\n")
        self._f.flush()

    def add(self, label: str, *, commanded_ticks: dict[str, int] | None, before: ArmSnapshot | None,
            after: ArmSnapshot | None, settle: dict | None = None, extra: dict | None = None,
            camera: dict | None = None) -> None:  # fmt: skip
        self._write({"type": "pose", "index": self.count, "label": label, "commanded_ticks": commanded_ticks,
                     "snapshot_before": snapshot_dict(before), "snapshot_after": snapshot_dict(after),
                     "settle": settle, "extra": extra or {}, "camera": camera})  # fmt: skip
        self.count += 1

    def close(self) -> None:
        if not self._f.closed:
            self._write({"type": "footer", "poses": self.count, "closed_monotonic_ns": time.monotonic_ns()})
            self._f.close()


def read_manifest(path) -> tuple[dict, list[dict]]:
    lines = [json.loads(x) for x in open(path)]
    return lines[0], [x for x in lines if x.get("type") == "pose"]


def capture_times(run_dir) -> dict[str, float]:
    """Camera runs: image name -> seconds since the first capture request."""
    t = {}
    for m in read_manifest(Path(run_dir) / "manifest.jsonl")[1]:
        if m["camera"]["ok"]:
            for f in m["camera"]["files"]:
                t[Path(f).name] = m["camera"]["t_request_ns"]
    t0 = min(t.values())
    return {k: (v - t0) / 1e9 for k, v in t.items()}
