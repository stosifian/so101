"""The MuJoCo SO-101 model (vendored Menagerie, models/so101_menagerie): joint ranges, FK, geometry.

Used for the step 7 mapping check and as the reference the step 8 parameterized FK is tested against.
MuJoCo here is a kinematics/geometry library: nothing is simulated or actuated.
"""

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


@dataclass
class VisualMesh:
    geom_id: int
    vertices: np.ndarray  # (n, 3) in the geom frame, metres
    faces: np.ndarray  # (m, 3) int
    rgba: np.ndarray


class ArmModel:
    def __init__(self, path: Path):
        self.m = mujoco.MjModel.from_xml_path(str(path))
        self.d = mujoco.MjData(self.m)
        self.qadr = {j: self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JOINTS}
        self.ranges = {j: tuple(self.m.jnt_range[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)])
                       for j in JOINTS}  # fmt: skip
        self.tool_site = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")

    def set_q(self, q: dict[str, float]) -> None:
        for j, v in q.items():
            self.d.qpos[self.qadr[j]] = v
        mujoco.mj_kinematics(self.m, self.d)

    def tool_pos(self) -> np.ndarray:
        return self.d.site_xpos[self.tool_site].copy()

    def geom_pose(self, g: int) -> tuple[np.ndarray, np.ndarray]:
        return self.d.geom_xpos[g].copy(), self.d.geom_xmat[g].reshape(3, 3).copy()

    def visual_meshes(self) -> list[VisualMesh]:
        out = []
        for g in range(self.m.ngeom):
            mid = self.m.geom_dataid[g]
            if self.m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or self.m.geom_group[g] != 2 or mid < 0:
                continue
            va, vn = self.m.mesh_vertadr[mid], self.m.mesh_vertnum[mid]
            fa, fn = self.m.mesh_faceadr[mid], self.m.mesh_facenum[mid]
            mat = self.m.geom_matid[g]
            rgba = self.m.mat_rgba[mat] if mat >= 0 else self.m.geom_rgba[g]
            out.append(VisualMesh(g, self.m.mesh_vert[va:va + vn].copy(), self.m.mesh_face[fa:fa + fn].copy(), rgba.copy()))
        return out
