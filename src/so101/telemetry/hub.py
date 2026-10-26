"""Telemetry pub/sub: one producer (the poller), many consumers (parquet, TUI, Rerun, later the watchdog).

Each subscriber gets its own bounded queue. When a queue is full the *oldest* sample is dropped
and counted, so a slow consumer loses data visibly and never stalls the poller or other consumers.
"""

import threading
from collections import deque
from dataclasses import dataclass

from so101.hw.arm import ArmSnapshot


@dataclass(frozen=True)
class Sample:
    arm: str
    seq: int  # per-producer counter; gaps never happen at the source, so gaps downstream = drops
    snapshot: ArmSnapshot
    goals: dict[str, int] | None = None  # Goal_Position commanded this cycle (SafeArm), after clamps


class Subscription:
    def __init__(self, name: str, maxlen: int):
        self.name = name
        self.dropped = 0
        self._q: deque[Sample] = deque(maxlen=maxlen)
        self._cv = threading.Condition()

    def _put(self, sample: Sample) -> None:
        with self._cv:
            if len(self._q) == self._q.maxlen:
                self.dropped += 1
            self._q.append(sample)
            self._cv.notify()

    def get(self, timeout: float | None = None) -> Sample | None:
        """Oldest pending sample, or None after `timeout` seconds."""
        with self._cv:
            if not self._q and not self._cv.wait_for(lambda: self._q, timeout):
                return None
            return self._q.popleft()

    def drain(self) -> list[Sample]:
        """Everything pending, oldest first, without waiting."""
        with self._cv:
            items = list(self._q)
            self._q.clear()
            return items


class Hub:
    def __init__(self):
        self._subs: list[Subscription] = []
        self._lock = threading.Lock()

    def subscribe(self, name: str, maxlen: int = 1000) -> Subscription:
        sub = Subscription(name, maxlen)
        with self._lock:
            self._subs.append(sub)
        return sub

    def unsubscribe(self, sub: Subscription) -> None:
        with self._lock:
            self._subs.remove(sub)

    def publish(self, sample: Sample) -> None:
        with self._lock:
            subs = list(self._subs)
        for sub in subs:
            sub._put(sample)

    @property
    def subscriptions(self) -> list[Subscription]:
        with self._lock:
            return list(self._subs)
