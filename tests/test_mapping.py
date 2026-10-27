import math

import pytest

from so101.kinematics.mapping import JointMap, load_mapping
from so101.kinematics.model import JOINTS, ArmModel

MAP = load_mapping("follower")
MODEL = ArmModel(MAP.model_path)


def test_roundtrip_and_scale():
    jm = JointMap(sign=-1, zero_ticks=2048)
    assert jm.to_rad(2048 + 1024) == pytest.approx(-math.pi / 2)  # quarter turn
    assert jm.to_ticks(jm.to_rad(1234)) == pytest.approx(1234)


def test_config_covers_every_model_joint():
    assert set(MAP.joints) == set(JOINTS) and MAP.model_path.exists()


def test_model_loads_with_expected_ranges():
    assert MODEL.ranges["shoulder_pan"] == pytest.approx((-1.91986, 1.91986))
    assert MODEL.ranges["gripper"] == pytest.approx((-0.174533, 1.7453292))


def test_rest_pose_is_plausible_and_a_bad_mapping_is_caught():
    rest = {"shoulder_pan": 2207, "shoulder_lift": 1007, "elbow_flex": 2928, "wrist_flex": 2853,
            "wrist_roll": 1169, "gripper": 2143}  # fmt: skip
    assert MAP.check(rest, MODEL.ranges) == {}
    flipped = type(MAP)({**MAP.joints, "elbow_flex": JointMap(-1, 1000)}, MAP.model_path, MAP.status, MAP.path)
    assert "elbow_flex" in flipped.check(rest, MODEL.ranges)


def test_fk_moves_tool_when_lift_moves():
    MODEL.set_q({j: 0.0 for j in JOINTS})
    p0 = MODEL.tool_pos()
    MODEL.set_q({**{j: 0.0 for j in JOINTS}, "shoulder_lift": 0.3})
    assert abs(MODEL.tool_pos() - p0).max() > 0.02
