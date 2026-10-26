from so101.analysis.limits import drop_spikes, propose, yaml_snippet


def test_sweep_gives_margined_limits():
    pos = list(range(1000, 3001, 10)) + list(range(3000, 999, -10))
    l = propose(pos, margin=50)
    assert l.status == "ok"
    assert (l.raw_min, l.raw_max, l.proposed) == (1000, 3000, (1050, 2950))


def test_isolated_spike_does_not_set_a_limit():
    pos = [2000, 2005, 2010, 3900, 2015, 2020] + list(range(2020, 1500, -10))
    kept, dropped = drop_spikes(pos)
    assert dropped == 1 and 3900 not in kept
    assert propose(pos, 50).raw_max == 2020


def test_fast_but_continuous_motion_is_not_a_spike():
    pos = [2000, 2050, 2100, 2150]  # 50 ticks/sample, steady
    assert drop_spikes(pos) == (pos, 0)


def test_wrap_through_zero_is_flagged():
    l = propose([100, 50, 10, 4090, 4050, 4000], margin=50)
    assert l.status == "wrapped" and l.proposed is None


def test_unmoved_joint_is_not_swept():
    l = propose([2047, 2048, 2047] * 10, margin=50)
    assert l.status == "not swept" and l.proposed is None


def test_yaml_snippet_comments_out_missing_limits():
    s = yaml_snippet("run1", 50, {"pan": propose(list(range(1000, 3000)), 50), "roll": propose([10, 4090], 50)})
    assert "  pan: [1050, 2949]" in s and "# roll: none (wrapped)" in s
