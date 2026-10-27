"""Collision model vs real recorded poses (ticks from the step 6 runs)."""

import pytest

from so101.config import load_arm_config
from so101.kinematics.collision import CollisionModel
from so101.kinematics.mapping import load_mapping
from so101.safety import SafetyConfig

MAP = load_mapping()
SAFETY = SafetyConfig.from_arm_config(load_arm_config("follower"))
CM = CollisionModel.from_safety(MAP, SAFETY)
CM_NO_MOUNT = CollisionModel(MAP, include_camera_mount=False)

REST_TODAY = dict(shoulder_pan=2207, shoulder_lift=1007, elbow_flex=2928, wrist_flex=2853, wrist_roll=1169, gripper=2143)
REST_YESTERDAY = dict(shoulder_pan=2176, shoulder_lift=1121, elbow_flex=2927, wrist_flex=2945, wrist_roll=1197, gripper=2126)

TABLE_CONTACTS = {  # seen by the user
    "wrist->table onset (20261026-162655, lift 1398)": dict(shoulder_pan=2289, shoulder_lift=1398, elbow_flex=2855,
                                                            wrist_flex=2851, wrist_roll=1166, gripper=2146),
    "wrist->table peak (20261026-162655, lift 1626)": dict(shoulder_pan=2289, shoulder_lift=1626, elbow_flex=2836,
                                                           wrist_flex=2847, wrist_roll=1166, gripper=2146),
    "gripper->table (20261027-091714, lift 1344)": dict(shoulder_pan=2206, shoulder_lift=1344, elbow_flex=2915,
                                                        wrist_flex=2902, wrist_roll=1167, gripper=2143),
}  # fmt: skip
NORMAL = {  # no contact
    "rest today": REST_TODAY,
    "rest yesterday": REST_YESTERDAY,
    "lift 1673, elbow unfolded (20261027-092126)": dict(shoulder_pan=2206, shoulder_lift=1673, elbow_flex=1036,
                                                        wrist_flex=1111, wrist_roll=1166, gripper=2143),
    "lift 1973, elbow 728 (20261027-092438)": dict(shoulder_pan=2206, shoulder_lift=1973, elbow_flex=728,
                                                   wrist_flex=1111, wrist_roll=1166, gripper=2143),
    "elbow sweep far end 2258": dict(shoulder_pan=2289, shoulder_lift=1074, elbow_flex=2258, wrist_flex=2853,
                                     wrist_roll=1166, gripper=2146),
    "wrist_flex sweep far end 2253": dict(shoulder_pan=2289, shoulder_lift=1074, elbow_flex=2927, wrist_flex=2253,
                                          wrist_roll=1166, gripper=2146),
    "pan sweep 1578 at rest": {**REST_TODAY, "shoulder_pan": 1578},
    "pan sweep 2795 at rest": {**REST_TODAY, "shoulder_pan": 2795},
    "gripper sweep open 3106": {**REST_TODAY, "gripper": 3106},
}  # fmt: skip


def test_config_values():
    assert (SAFETY.collision_table_margin_m, SAFETY.collision_self_margin_m) == (0.003, 0.005)
    assert SAFETY.collision_include_camera_mount


@pytest.mark.parametrize("name", list(TABLE_CONTACTS))
def test_real_table_contacts_are_flagged(name):
    r = CM.check_ticks(TABLE_CONTACTS[name])
    assert not r.ok and r.min_table < 0, (name, r.table)


@pytest.mark.parametrize("name", list(NORMAL))
def test_normal_poses_are_allowed(name):
    """Every normal pose is either clear of the 3 mm margin, or (rest yesterday, +2.3 mm) allowed as a start
    pose by the margin rule: inside the margin is fine if not getting closer."""
    r = CM.check_ticks(NORMAL[name])
    assert CM.table_violations(r, ref=r) == [], name
    assert r.min_table > 0  # never penetrating


def test_margin_rule_allows_moving_away_but_not_closer():
    rest = CM.check_ticks(REST_YESTERDAY)
    assert rest.min_table < 0.003  # inside the margin at rest
    # near rest, raising lift (+) swings the folded gripper DOWN toward the table (as in the real contacts)
    away = CM.check_ticks({**REST_YESTERDAY, "elbow_flex": REST_YESTERDAY["elbow_flex"] - 25})
    closer = CM.check_ticks({**REST_YESTERDAY, "shoulder_lift": REST_YESTERDAY["shoulder_lift"] + 25})
    assert away.min_table > rest.min_table and CM.table_violations(away, ref=rest) == []
    assert closer.min_table < rest.min_table and CM.table_violations(closer, ref=rest) != []
    assert CM.table_violations(closer, ref=None) != []  # no reference: plain margin


def test_known_miss_gripper_vs_shoulder_in_rest_pose():
    """Real contact at wrist_roll ~2645 in rest pose (20261026-160843), WITHOUT the camera mount. The model puts
    gripper and shoulder ~13-14 mm apart: within model + by-eye-calibration uncertainty. Documented blind spot;
    covered by the wrist_roll limit (2550), contact detection and the operator. If this starts failing, the
    model or calibration got better: update the docs and consider enforcing self-collision."""
    r = CM_NO_MOUNT.check_ticks({**REST_TODAY, "wrist_roll": 2645})
    d = r.self_pairs[("gripper", "shoulder")]
    assert 0.010 < d < 0.020  # model says ~13-14 mm apart where the real parts touched
    assert not any("gripper ~ shoulder" in w for w in r.warnings)


def test_camera_mount_adds_shapes_and_adjacent_pairs_are_never_checked():
    assert len(CM.geoms) > len(CM_NO_MOUNT.geoms)
    m = CM.m
    for g1, g2 in CM.pairs:
        b1, b2 = m.geom_bodyid[g1], m.geom_bodyid[g2]
        assert b1 != b2 and m.body_parentid[b1] != b2 and m.body_parentid[b2] != b1


def test_path_check_returns_first_violation():
    path = [MAP.to_q(NORMAL["lift 1673, elbow unfolded (20261027-092126)"]),
            MAP.to_q(TABLE_CONTACTS["gripper->table (20261027-091714, lift 1344)"]),
            MAP.to_q(TABLE_CONTACTS["wrist->table peak (20261026-162655, lift 1626)"])]  # fmt: skip
    i, r = CM.check_path(path)
    assert i == 1 and not r.ok
    assert CM.check_path(path[:1]) is None


def test_disabled_table_check_never_violates():
    cm = CollisionModel(MAP, table_margin=None)
    assert cm.check_ticks(TABLE_CONTACTS["wrist->table peak (20261026-162655, lift 1626)"]).ok
