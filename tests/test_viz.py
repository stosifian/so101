from so101.hw.arm import ArmSnapshot, JointState
from so101.telemetry.hub import Hub, Sample
from so101.telemetry.viz import RerunView


def test_writes_replayable_rrd_without_viewer(tmp_path):
    hub = Hub()
    rrd = tmp_path / "telemetry.rrd"
    view = RerunView(hub.subscribe("viz"), ["follower"], rrd, "test-run", t0_ns=0, spawn=False)
    for seq in range(50):
        state = JointState(2000 + seq, 10, -5, 119, 30, 0, 0, 2)
        hub.publish(Sample("follower", seq, ArmSnapshot(seq * 10_000_000, seq * 10_000_000 + 1, {"gripper": state})))
    hub.publish(Sample("follower", 50, ArmSnapshot(1, 2, {"gripper": None}, {"gripper": "no status packet"})))
    view.update()
    view.close()
    assert view.samples == 51
    assert rrd.exists() and rrd.stat().st_size > 1000
