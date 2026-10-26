from so101.hw.bus import MockBus
from so101.hw.dump import dump_arm

JOINTS = {"shoulder_pan": 1, "shoulder_lift": 2, "gripper": 6}


def test_dump_reads_all_responding_servos():
    bus = MockBus.with_servos([1, 2, 6], voltage_dv=119)
    d = dump_arm(bus, JOINTS)
    pan = d["servos"]["shoulder_pan"]
    assert pan["responded"]
    assert pan["registers"]["Present_Voltage"]["value"] == 119
    assert pan["registers"]["Model"]["value"] == 777
    assert pan["errors"] == {}


def test_dump_marks_missing_servo_without_crashing():
    bus = MockBus.with_servos([1, 2])  # id 6 absent
    d = dump_arm(bus, JOINTS)
    assert d["servos"]["gripper"]["responded"] is False
    assert d["servos"]["shoulder_pan"]["responded"] is True
