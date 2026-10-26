import time

import pytest

from so101.hw.arm import Arm, ArmSnapshot
from so101.hw.bus import MockBus
from so101.telemetry.hub import Hub, Sample
from so101.telemetry.poller import Poller

JOINTS = {"shoulder_pan": 1, "gripper": 6}


def _sample(seq: int) -> Sample:
    return Sample("follower", seq, ArmSnapshot(seq, seq, {}))


def test_every_subscriber_gets_every_sample():
    hub = Hub()
    a, b = hub.subscribe("a"), hub.subscribe("b")
    for i in range(5):
        hub.publish(_sample(i))
    assert [s.seq for s in a.drain()] == list(range(5))
    assert [s.seq for s in b.drain()] == list(range(5))


def test_full_queue_drops_oldest_and_counts():
    hub = Hub()
    slow = hub.subscribe("slow", maxlen=3)
    for i in range(10):
        hub.publish(_sample(i))
    assert [s.seq for s in slow.drain()] == [7, 8, 9]
    assert slow.dropped == 7


def test_get_times_out_when_empty():
    sub = Hub().subscribe("x")
    t0 = time.monotonic()
    assert sub.get(timeout=0.05) is None
    assert time.monotonic() - t0 >= 0.04


def test_poller_publishes_at_roughly_the_requested_rate():
    hub = Hub()
    sub = hub.subscribe("t", maxlen=10_000)
    with Poller(Arm(MockBus.with_servos([1, 6]), JOINTS), hub, "follower", rate_hz=200) as p:
        time.sleep(0.5)
    samples = sub.drain()
    assert 70 <= len(samples) <= 110  # ~100 expected; loose bounds for a busy machine
    assert [s.seq for s in samples] == list(range(len(samples)))  # no gaps at the source
    assert p.stats.failed_snapshots == 0
    assert p.error is None and not p.alive


class SlowBus(MockBus):
    def read_block(self, servo_id, addr, size):
        time.sleep(0.004)
        return super().read_block(servo_id, addr, size)


def test_poller_counts_overruns_and_skips_instead_of_bursting():
    bus = SlowBus.with_servos([1, 6])  # 2 servos x 4 ms = 8 ms per snapshot
    hub = Hub()
    with Poller(Arm(bus, JOINTS), hub, "follower", rate_hz=500) as p:  # 2 ms period
        time.sleep(0.2)
    assert p.stats.overruns > 0
    assert p.stats.skipped_slots > 0
    assert p.stats.max_read_ms >= 8


def test_poller_reports_failing_servo_without_dying():
    hub = Hub()
    with Poller(Arm(MockBus.with_servos([1]), JOINTS), hub, "follower", rate_hz=200) as p:  # id 6 missing
        time.sleep(0.05)
    assert p.stats.cycles > 0
    assert p.stats.failed_snapshots == p.stats.cycles
    assert p.error is None


def test_poller_records_thread_death():
    class Unplugged(MockBus):
        def read_block(self, *a):
            raise OSError("device disconnected")

    p = Poller(Arm(Unplugged.with_servos([1]), JOINTS), Hub(), "follower").start()
    time.sleep(0.05)
    assert not p.alive
    assert isinstance(p.error, OSError)


def test_rate_must_be_positive():
    with pytest.raises(ValueError):
        Poller(Arm(MockBus(), {}), Hub(), "x", rate_hz=0)


def test_bus_has_one_owner_at_a_time():
    from so101.hw.bus import BusError

    bus = MockBus.with_servos([1, 6])
    bus.claim("safearm")
    with pytest.raises(BusError, match="already owned by 'safearm'"):
        Poller(Arm(bus, JOINTS), Hub(), "follower").start()
    with pytest.raises(BusError):
        bus.release("someone-else")
    bus.release("safearm")
    assert bus.owner is None


def test_poller_releases_bus_when_stopped_or_dead():
    bus = MockBus.with_servos([1, 6])
    with Poller(Arm(bus, JOINTS), Hub(), "follower") as p:
        assert bus.owner == "poller-follower"
    assert bus.owner is None
    bus.claim("safearm")  # free again for a motion loop

    class Unplugged(MockBus):
        def read_block(self, *a):
            raise OSError("device disconnected")

    dead = Unplugged.with_servos([1])
    Poller(Arm(dead, JOINTS), Hub(), "follower").start()
    time.sleep(0.05)
    assert dead.owner is None
