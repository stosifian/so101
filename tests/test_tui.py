from rich.console import Console

from so101.hw.arm import ArmSnapshot, JointState
from so101.telemetry.hub import Hub, Sample
from so101.telemetry.tui import Tui


def _state(pos: int, v: int = 119, t: int = 30) -> JointState:
    return JointState(pos, 0, 0, v, t, 0, 0, 0)


def _publish(hub: Hub, seq: int, pos: int, **kw) -> None:
    snap = ArmSnapshot(seq * 10_000_000, seq * 10_000_000 + 1, {"gripper": _state(pos, **kw)})
    hub.publish(Sample("follower", seq, snap))


def test_tracks_min_max_and_seq_gaps():
    hub = Hub()
    tui = Tui(hub.subscribe("tui"), {"follower": (11.0, 13.0)})
    for seq, pos in [(0, 2000), (1, 2100), (3, 1900)]:  # seq 2 missing
        _publish(hub, seq, pos)
    tui.update()
    view = tui.arms["follower"]
    g = view.joints["gripper"]
    assert (g.pos_min, g.pos_max, g.last.position) == (1900, 2100, 1900)
    assert view.seq_gaps == 1
    assert view.rate_hz > 0


def test_median_hides_single_glitch():
    hub = Hub()
    tui = Tui(hub.subscribe("tui"), {"follower": (11.0, 13.0)})
    for seq in range(20):
        _publish(hub, seq, 2000, t=43 if seq == 10 else 30, v=104 if seq == 11 else 119)
    tui.update()
    out = Console(width=200, record=True)
    out.print(tui.render())
    text = out.export_text()
    assert "43" not in text and "10.4" not in text
    assert "11.9" in text and "30" in text


def test_missing_joint_renders_error():
    hub = Hub()
    tui = Tui(hub.subscribe("tui"), {"follower": (11.0, 13.0)})
    hub.publish(Sample("follower", 0, ArmSnapshot(0, 1, {"gripper": None}, {"gripper": "no status packet"})))
    tui.update()
    out = Console(width=200, record=True)
    out.print(tui.render())
    assert "no status packet" in out.export_text()
