"""Ticks <-> model joint angles (step 7). The only place raw ticks become radians (CLAUDE.md convention).

q_rad = sign * (ticks - zero_ticks) * 2*pi / 4096, per joint, from config/calibration/<arm>_mapping.yaml.
The model (models/so101_menagerie) puts joint zero at LeRobot's "middle of range" pose. Whether our
zero/sign match is verified by placing the arm in known poses (scripts/readonly/model_check.py).
`check()` is the "refuse to start" test: a wrong mapping usually puts some joint outside its model range.
"""

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from so101.config import CONFIG_DIR, REPO_ROOT

TICKS_PER_REV = 4096
RAD_PER_TICK = 2 * math.pi / TICKS_PER_REV


@dataclass(frozen=True)
class JointMap:
    sign: int
    zero_ticks: float

    def to_rad(self, ticks: float) -> float:
        return self.sign * (ticks - self.zero_ticks) * RAD_PER_TICK

    def to_ticks(self, rad: float) -> float:
        return self.zero_ticks + self.sign * rad / RAD_PER_TICK


@dataclass(frozen=True)
class Mapping:
    joints: dict[str, JointMap]
    model_path: Path
    status: str  # hypothesis | verified
    path: Path

    def to_q(self, ticks: dict[str, float]) -> dict[str, float]:
        return {j: self.joints[j].to_rad(t) for j, t in ticks.items() if j in self.joints}

    def to_ticks(self, q: dict[str, float]) -> dict[str, float]:
        return {j: self.joints[j].to_ticks(r) for j, r in q.items() if j in self.joints}

    def check(self, ticks: dict[str, float], ranges: dict[str, tuple[float, float]],
              margin_rad: float = math.radians(10)) -> dict[str, float]:  # fmt: skip
        """Joints whose mapped angle lies outside the model range +- margin: {joint: q_rad}. Empty = plausible."""
        out = {}
        for j, q in self.to_q(ticks).items():
            lo, hi = ranges[j]
            if not lo - margin_rad <= q <= hi + margin_rad:
                out[j] = q
        return out


def load_mapping(arm: str = "follower", config_dir: Path = CONFIG_DIR) -> Mapping:
    path = config_dir / "calibration" / f"{arm}_mapping.yaml"
    data = yaml.safe_load(path.read_text())
    joints = {}
    for j, v in data["joints"].items():
        if v["sign"] not in (1, -1):
            raise ValueError(f"{path}: {j}.sign must be +1 or -1")
        joints[j] = JointMap(int(v["sign"]), float(v["zero_ticks"]))
    return Mapping(joints, REPO_ROOT / data["model"], data.get("status", "hypothesis"), path)
