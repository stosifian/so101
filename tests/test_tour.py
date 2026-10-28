import json

from so101.config import REPO_ROOT, load_arm_config
from so101.hw.arm import Arm
from so101.hw.sim import SimBus
from so101.kinematics.collision import CollisionModel
from so101.kinematics.fk import CHAIN_JOINTS, chain_from_mujoco
from so101.kinematics.ik import limits_rad
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.motion.tour import Waypoint, load_tour, plan_tour
from so101.safety import SafetyConfig

CFG = load_arm_config("follower")
SAFETY = SafetyConfig.from_arm_config(CFG)
MAP = load_mapping("follower")
CHAIN = chain_from_mujoco(ArmModel(MAP.model_path).m)
COL = CollisionModel.from_safety(MAP, SAFETY)
REST = {j: s.position for j, s in Arm(SimBus.from_dump(json.loads(
    (REPO_ROOT / "tests/fixtures/follower_factory_dump.json").read_text())), CFG.joints).snapshot().joints.items()}  # fmt: skip


def plan(wps):
    return plan_tour(wps, REST, chain=CHAIN, mapping=MAP, limits_ticks=SAFETY.limits,
                     limits_rad=limits_rad(MAP, SAFETY.limits, CHAIN_JOINTS), collision=COL)  # fmt: skip


def test_default_tour_plans_from_rest():
    wps, _ = load_tour(REPO_ROOT / "config/tours/pick_and_place.yaml")
    p = plan(wps)
    assert p.ok, p.reason
    assert len(p.steps) == len(wps)
    park = p.steps[-1].segment.target
    assert park["elbow_flex"] == SAFETY.limits["elbow_flex"][1]  # rest elbow is outside the limit: clamped
    assert p.phase_fn()(0.0)[1] == "ready" and p.goal_fn()(1e9) == park


def test_refuses_a_target_through_the_table():
    p = plan([Waypoint("ready", pos_m=[0.25, 0, 0.14], approach=[0.707, 0, -0.707]),
              Waypoint("into_table", pos_m=[0.25, 0, 0.0], approach=[0.707, 0, -0.707])])  # fmt: skip
    assert not p.ok and p.reason.startswith("into_table") and len(p.steps) == 1


def test_refuses_unreachable_and_out_of_limits():
    assert not plan([Waypoint("too_far", pos_m=[0.8, 0, 0.1])]).ok
    p = plan([Waypoint("bad_roll", ticks={"wrist_roll": 3000})])
    assert not p.ok and "outside limits" in p.reason
