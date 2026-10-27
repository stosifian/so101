"""Collision model (step 8b): a geometry check on joint angles with MuJoCo. No physics is simulated.

Given joint angles (from the real arm via the step 7 mapping, or planned goals), MuJoCo places the vendored
model's collision shapes and measures signed distances (negative = overlap):
- table: every collision shape vs the table plane, the base underside z = -2.4 mm (the base sits flat on
  the table; plan D12). Below `table_margin` is a VIOLATION (step 9: refuse a plan / freeze at runtime).
- self: closest distance between non-adjacent links. Below `self_margin` is a WARNING only.

What it resolves (tested on real runs, tests/test_collision.py):
- the three real table contacts show clear penetration (-2 to -19 mm); normal sweeps are >= 27 mm clear;
- NOT the gripper-base vs shoulder contact in the folded rest pose (wrist_roll ~2645): the model puts them
  13-14 mm apart, within model + by-eye-calibration uncertainty. Covered instead by the conservative
  wrist_roll limit, contact detection and the operator. Revisit after 8c / Phase 2 calibration.
LeRobot has no collision checking; the SO-ARM community direction is MuJoCo (robocurve #26). Validation only:
no collision-avoiding planning in Phase 1 (plan guardrail).
"""

from dataclasses import dataclass, field

import mujoco
import numpy as np

from so101.kinematics.mapping import Mapping
from so101.kinematics.model import JOINTS

TABLE_Z = -0.0024  # base underside in the model frame (metres)
# Bodies that sit together by design (the mount is bolted onto the gripper next to the moving jaw).
DESIGN_PAIRS = {frozenset({"camera_mount", "moving_jaw_so101_v1"}), frozenset({"camera_mount", "wrist"})}


@dataclass
class CollisionReport:
    table: dict[str, float] = field(default_factory=dict)  # body -> distance to table (m), closest first
    self_pairs: dict[tuple[str, str], float] = field(default_factory=dict)  # body pair -> distance (m)
    violations: list[str] = field(default_factory=list)  # table: bodies below table_margin
    warnings: list[str] = field(default_factory=list)  # self: pairs below self_margin

    @property
    def ok(self) -> bool:
        return not self.violations

    @property
    def min_table(self) -> float:
        return min(self.table.values(), default=float("inf"))


class CollisionModel:
    def __init__(self, mapping: Mapping, *, table_z: float = TABLE_Z, table_margin: float | None = 0.0,
                 self_margin: float | None = 0.005, include_camera_mount: bool = True, query_m: float = 0.3):  # fmt: skip
        self.mapping = mapping
        self.table_margin, self.self_margin, self.query = table_margin, self_margin, query_m
        spec = mujoco.MjSpec.from_file(str(mapping.model_path.parent / "so101.xml"))  # vendored file, unmodified
        g = spec.worldbody.add_geom()
        g.name, g.type, g.size, g.pos = "table", mujoco.mjtGeom.mjGEOM_PLANE, [0, 0, 0.01], [0, 0, table_z]
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        self.table_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "table")
        self.qadr = {j: self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JOINTS}

        def body(g: int) -> str:
            return mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY, self.m.geom_bodyid[g])

        self.body = body
        self.geoms = [g for g in range(self.m.ngeom)
                      if (self.m.geom_contype[g] or self.m.geom_conaffinity[g]) and g != self.table_id
                      and (include_camera_mount or body(g) != "camera_mount")]  # fmt: skip
        parent = self.m.body_parentid
        pairs = []
        for i, g1 in enumerate(self.geoms):
            for g2 in self.geoms[i + 1:]:
                b1, b2 = self.m.geom_bodyid[g1], self.m.geom_bodyid[g2]
                if b1 == b2 or parent[b1] == b2 or parent[b2] == b1:
                    continue  # same or adjacent links: always touching at the joint
                if frozenset({body(g1), body(g2)}) in DESIGN_PAIRS:
                    continue
                pairs.append((g1, g2))
        self.pairs = pairs
        self._ft = np.zeros(6)

    @classmethod
    def from_safety(cls, mapping: Mapping, safety) -> "CollisionModel":
        return cls(mapping, table_margin=safety.collision_table_margin_m, self_margin=safety.collision_self_margin_m,
                   include_camera_mount=safety.collision_include_camera_mount)  # fmt: skip

    def _dist(self, g1: int, g2: int) -> float:
        return mujoco.mj_geomDistance(self.m, self.d, g1, g2, self.query, self._ft)

    def check(self, q: dict[str, float]) -> CollisionReport:
        """q: joint angles in radians (any subset of JOINTS; missing joints stay at their last value)."""
        for j, v in q.items():
            self.d.qpos[self.qadr[j]] = v
        mujoco.mj_kinematics(self.m, self.d)
        r = CollisionReport()
        table: dict[str, float] = {}
        for g in self.geoms:
            b = self.body(g)
            table[b] = min(table.get(b, np.inf), self._dist(g, self.table_id))
        r.table = dict(sorted(table.items(), key=lambda kv: kv[1]))
        selfd: dict[tuple[str, str], float] = {}
        for g1, g2 in self.pairs:
            k = tuple(sorted((self.body(g1), self.body(g2))))
            selfd[k] = min(selfd.get(k, np.inf), self._dist(g1, g2))
        r.self_pairs = dict(sorted(selfd.items(), key=lambda kv: kv[1]))
        if self.table_margin is not None:
            r.violations = [f"{b} {d * 1000:+.1f} mm to table (margin {self.table_margin * 1000:.0f} mm)"
                            for b, d in r.table.items() if d < self.table_margin]  # fmt: skip
        if self.self_margin is not None:
            r.warnings = [f"{a} ~ {b} {d * 1000:+.1f} mm (margin {self.self_margin * 1000:.0f} mm, warn-only)"
                          for (a, b), d in r.self_pairs.items() if d < self.self_margin]  # fmt: skip
        return r

    def check_ticks(self, ticks: dict[str, float]) -> CollisionReport:
        return self.check(self.mapping.to_q(ticks))

    def check_path(self, qs: list[dict[str, float]]) -> tuple[int, CollisionReport] | None:
        """First pose along a path with a table violation, or None if the whole path is clear."""
        for i, q in enumerate(qs):
            r = self.check(q)
            if not r.ok:
                return i, r
        return None
