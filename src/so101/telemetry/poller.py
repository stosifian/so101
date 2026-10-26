"""Background poller: snapshot an arm at a fixed rate and publish every sample to a Hub.

Pollers are for read-only sessions only. A Poller claims its arm's bus on start() and releases
it when its thread ends. In motion sessions (step 4+) no poller runs on the follower: SafeArm's
own loop claims the bus, reads, checks, writes, and publishes the same Samples to the Hub.

Timing uses absolute monotonic deadlines (next = previous + period), so the rate doesn't drift.
A cycle that overruns its period is counted; if we fall more than a full period behind, the
schedule resets to now instead of bursting to catch up, and the skipped slots are counted.
"""

import threading
import time
from dataclasses import dataclass

from so101.hw.arm import Arm
from so101.telemetry.hub import Hub, Sample


@dataclass
class PollerStats:
    cycles: int = 0
    failed_snapshots: int = 0  # at least one servo didn't answer
    overruns: int = 0  # snapshot finished after the next deadline
    skipped_slots: int = 0  # deadlines dropped after falling > 1 period behind
    max_read_ms: float = 0.0


class Poller:
    def __init__(self, arm: Arm, hub: Hub, name: str, rate_hz: float = 100.0):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0")
        self.arm, self.hub, self.name = arm, hub, name
        self.period_ns = int(1e9 / rate_hz)
        self.stats = PollerStats()
        self.error: BaseException | None = None  # set if the thread died
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"poller-{name}", daemon=True)

    def start(self) -> "Poller":
        self.arm.bus.claim(self._thread.name)  # raises BusError if someone else owns the bus
        self._thread.start()
        return self

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        self._thread.join(timeout)

    @property
    def alive(self) -> bool:
        return self._thread.is_alive()

    def __enter__(self) -> "Poller":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def _run(self) -> None:
        try:
            next_t = time.monotonic_ns()
            while not self._stop.is_set():
                snap = self.arm.snapshot()
                self.hub.publish(Sample(self.name, self.stats.cycles, snap))
                s = self.stats
                s.cycles += 1
                s.failed_snapshots += not snap.ok
                s.max_read_ms = max(s.max_read_ms, snap.duration_ms)

                next_t += self.period_ns
                now = time.monotonic_ns()
                if now > next_t:
                    s.overruns += 1
                    behind = (now - next_t) // self.period_ns
                    if behind >= 1:
                        s.skipped_slots += behind
                        next_t = now
                    continue
                self._stop.wait((next_t - now) / 1e9)
        except BaseException as e:  # e.g. serial port unplugged; consumers check .error / .alive
            self.error = e
        finally:
            self.arm.bus.release(self._thread.name)
