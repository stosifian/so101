"""Propose software joint limits from a torque-off, by-hand sweep.

Pattern: LeRobot's `SerialMotorsBus.record_ranges_of_motion` (src/lerobot/motors/motors_bus.py,
huggingface/lerobot @ 45cca1a, 2026-10-26): track min/max Present_Position while the user moves each
joint through its range by hand, torque off. What we change and why:
- No EEPROM writes. LeRobot's `write_calibration` writes Min/Max_Position_Limit (and Homing_Offset)
  to the servos, with no margin; we only propose
  software limits (hard rule), applied to config/arm_<name>.yaml after review.
- A margin inward from the recorded extremes, so the limit sits short of the hard stop.
- Wrap detection: a jump of more than half a turn between consecutive samples means the joint
  crossed 0/4095 (LeRobot treats wrist_roll as full-turn for this reason). No limits proposed then.
- Spike rejection: an isolated sample far from both neighbours is dropped. Position has been clean
  so far, but V/°C glitch (notes 2026-10-26) and a single bad sample would set a limit.
"""

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import pyarrow.parquet as pq

from so101.config import REPO_ROOT

DERIVED_DIR = REPO_ROOT / "data" / "derived"
HALF_TURN = 2048
SPIKE_TICKS = 100  # 100 ticks in one 10 ms sample = ~880 deg/s; a hand can't do that
MIN_RANGE = 200  # ~18 deg; less than this means the joint wasn't swept


@dataclass(frozen=True)
class JointLimits:
    samples: int
    spikes_dropped: int
    raw_min: int | None
    raw_max: int | None
    proposed: tuple[int, int] | None  # (min, max) ticks with margin, or None
    status: str  # ok | not swept | wrapped | no data


def drop_spikes(pos: list[int], spike: int = SPIKE_TICKS) -> tuple[list[int], int]:
    keep, dropped = [], 0
    for i, p in enumerate(pos):
        if 0 < i < len(pos) - 1:
            a, b = pos[i - 1], pos[i + 1]
            if abs(p - a) > spike and abs(p - b) > spike and abs(a - b) <= spike:
                dropped += 1
                continue
        keep.append(p)
    return keep, dropped


def propose(positions: list[int], margin: int) -> JointLimits:
    """positions: one joint's Present_Position samples in time order."""
    pos, spikes = drop_spikes(positions)
    if not pos:
        return JointLimits(0, spikes, None, None, None, "no data")
    lo, hi = min(pos), max(pos)
    if any(abs(b - a) > HALF_TURN for a, b in zip(pos, pos[1:])):
        return JointLimits(len(pos), spikes, lo, hi, None, "wrapped")
    if hi - lo < max(MIN_RANGE, 2 * margin + 1):
        return JointLimits(len(pos), spikes, lo, hi, None, "not swept")
    return JointLimits(len(pos), spikes, lo, hi, (lo + margin, hi - margin), "ok")


def propose_from_run(run_dir: Path, margin: int) -> dict[str, dict[str, JointLimits]]:
    """arm -> joint -> JointLimits, from a run's telemetry parquet (failed reads ignored)."""
    rows = pq.read_table(Path(run_dir) / "telemetry").to_pylist()
    series: dict[tuple[str, str], list[tuple[int, int]]] = {}
    for r in rows:
        if r["error"] is None:
            series.setdefault((r["arm"], r["joint"]), []).append((r["seq"], r["position"]))
    out: dict[str, dict[str, JointLimits]] = {}
    for (arm, joint), s in series.items():
        out.setdefault(arm, {})[joint] = propose([p for _, p in sorted(s)], margin)
    return out


def save_proposal(run_id: str, margin: int, limits: dict[str, dict[str, JointLimits]],
                  root: Path = DERIVED_DIR) -> Path:  # fmt: skip
    d = root / run_id
    d.mkdir(parents=True, exist_ok=True)
    p = d / "limits_proposed.json"
    body = {"source_run": run_id, "margin_ticks": margin,
            "arms": {a: {j: asdict(l) for j, l in js.items()} for a, js in limits.items()}}  # fmt: skip
    p.write_text(json.dumps(body, indent=2))
    return p


def yaml_snippet(run_id: str, margin: int, joints: dict[str, JointLimits]) -> str:
    lines = [f"limits_ticks:   # software limits, by-hand sweep {run_id}, margin {margin}"]
    for joint, l in joints.items():
        if l.proposed:
            lines.append(f"  {joint}: [{l.proposed[0]}, {l.proposed[1]}]")
        else:
            lines.append(f"  # {joint}: none ({l.status})")
    return "\n".join(lines)
