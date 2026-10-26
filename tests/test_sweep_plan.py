import pytest

from so101.config import load_arm_config
from so101.motion.sweep import generic_waypoints, plan_sweep, sweep_goals
from so101.safety import SafetyConfig

LIM = SafetyConfig.from_arm_config(load_arm_config("follower")).limits


def test_generic_pan_matches_the_outline():
    assert generic_waypoints(*LIM["shoulder_pan"], 0.5, 7) == [1578, 1781, 1984, 2186, 2389, 2592, 2795]


def test_generic_visits_from_nearest_end_out_and_back():
    p = plan_sweep("wrist_flex", 2853, *LIM["wrist_flex"], "generic")
    assert p.path[0] == 2480 and p.path[6] == 1491 and p.path[-1] == 2480  # nearest end first
    assert len(p.path) == 13 and p.home == 2853 and p.entry is None


def test_anchored_steps_toward_mid_range_and_back():
    p = plan_sweep("shoulder_lift", 1075, *LIM["shoulder_lift"], "anchored", step=150, steps=4)
    assert p.path == [1225, 1375, 1525, 1675, 1525, 1375, 1225] and p.home == 1075


def test_outside_start_enters_range_first_and_ends_there():
    p = plan_sweep("elbow_flex", 2926, *LIM["elbow_flex"], "anchored", step=150, steps=4)
    assert p.entry == 2858 and p.home == 2858
    assert p.path == [2708, 2558, 2408, 2258, 2408, 2558, 2708]


def test_waypoints_outside_limits_are_refused():
    with pytest.raises(ValueError, match="outside limits"):
        plan_sweep("shoulder_lift", 1075, *LIM["shoulder_lift"], "anchored", step=600, steps=4)


def test_goal_function_ramps_holds_and_returns_home():
    p = plan_sweep("shoulder_pan", 2000, *LIM["shoulder_pan"], "anchored", step=100, steps=2)  # mid-range 2186: go up
    fn, total = sweep_goals(p, speed=100, hold_s=1.0)
    # path 2100, 2200, 2100 then home 2000: 4 legs of 100 ticks at 100 t/s, each + 1 s hold
    assert total == pytest.approx(4 * (1.0 + 1.0))
    assert fn(0.5)["shoulder_pan"] == pytest.approx(2050)
    assert fn(1.5)["shoulder_pan"] == pytest.approx(2100)  # holding at the first waypoint
    assert fn(total + 1)["shoulder_pan"] == pytest.approx(2000)
