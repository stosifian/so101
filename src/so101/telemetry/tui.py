"""Rich live table: a Hub consumer that shows the latest state of every joint.

Display only; it never affects logging. Voltage and temperature are shown as a short running
median because IDs 4-6 report single-sample ADC glitches (~1 %, see notes 2026-10-26); the
parquet log keeps the raw values. Position min/max since start are tracked for limit recording.
"""

from collections import deque
from dataclasses import dataclass, field
from statistics import median

from rich.console import Group
from rich.table import Table
from rich.text import Text

from so101.hw.arm import JointState
from so101.telemetry.hub import Subscription

MEDIAN_WINDOW = 50  # samples (0.5 s at 100 Hz)
TEMP_WARN_C, TEMP_HOT_C = 50, 60  # display colours only; firmware trips at 70 (follower)


@dataclass
class JointView:
    last: JointState | None = None
    error: str | None = None
    pos_min: int | None = None
    pos_max: int | None = None
    volts: deque = field(default_factory=lambda: deque(maxlen=MEDIAN_WINDOW))
    temps: deque = field(default_factory=lambda: deque(maxlen=MEDIAN_WINDOW))

    def add(self, state: JointState | None, error: str | None) -> None:
        self.error = error
        if state is None:
            return
        self.last = state
        self.pos_min = state.position if self.pos_min is None else min(self.pos_min, state.position)
        self.pos_max = state.position if self.pos_max is None else max(self.pos_max, state.position)
        self.volts.append(state.voltage_dv)
        self.temps.append(state.temperature_c)


@dataclass
class ArmView:
    expected_voltage_v: tuple[float, float]
    joints: dict[str, JointView] = field(default_factory=dict)
    samples: int = 0
    last_seq: int = -1
    seq_gaps: int = 0
    recent_t_ns: deque = field(default_factory=lambda: deque(maxlen=200))  # for the live rate

    @property
    def rate_hz(self) -> float:
        t = self.recent_t_ns
        return (len(t) - 1) / ((t[-1] - t[0]) / 1e9) if len(t) > 1 and t[-1] > t[0] else 0.0


class Tui:
    def __init__(self, sub: Subscription, expected_voltage: dict[str, tuple[float, float]], title: str = ""):
        self.sub, self.title = sub, title
        self.arms = {name: ArmView(v) for name, v in expected_voltage.items()}
        self.footer: list[str] = []  # extra status lines set by the caller (poller/logger stats)

    def update(self) -> None:
        for sample in self.sub.drain():
            view = self.arms[sample.arm]
            if view.last_seq >= 0:
                view.seq_gaps += sample.seq - view.last_seq - 1
            view.last_seq = sample.seq
            view.samples += 1
            view.recent_t_ns.append(sample.snapshot.t_start_ns)
            for joint, state in sample.snapshot.joints.items():
                view.joints.setdefault(joint, JointView()).add(state, sample.snapshot.errors.get(joint))

    def _arm_table(self, name: str, view: ArmView) -> Table:
        t = Table(
            title=f"{name}  {view.rate_hz:5.1f} Hz  samples {view.samples}  seq gaps {view.seq_gaps}",
            title_justify="left",
        )
        for col in ["joint", "pos", "min", "max", "range", "speed", "load %", "V (med)", "°C (med)", "status",
                    "moving", "mA"]:  # fmt: skip
            t.add_column(col, justify="left" if col == "joint" else "right")
        lo_v, hi_v = view.expected_voltage_v
        for joint, jv in view.joints.items():
            s = jv.last
            if s is None:
                t.add_row(joint, Text(jv.error or "no data", style="red"))
                continue
            v = median(jv.volts) / 10
            temp = median(jv.temps)
            t.add_row(
                Text(joint, style="red" if jv.error else ""),
                str(s.position), str(jv.pos_min), str(jv.pos_max), str(jv.pos_max - jv.pos_min),
                str(s.speed), f"{s.load / 10:.1f}",
                Text(f"{v:.1f}", style="green" if lo_v <= v <= hi_v else "red"),
                Text(f"{temp:g}", style="red" if temp >= TEMP_HOT_C else "yellow" if temp >= TEMP_WARN_C else ""),
                Text(str(s.status), style="red" if s.status else ""),
                str(s.moving), f"{s.current_ma:.0f}",
            )  # fmt: skip
        return t

    def render(self) -> Group:
        parts = [Text(self.title, style="bold")] if self.title else []
        parts += [self._arm_table(name, view) for name, view in self.arms.items()]
        parts += [Text(line, style="dim") for line in [*self.footer, f"tui dropped: {self.sub.dropped}"]]
        return Group(*parts)
