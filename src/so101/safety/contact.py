"""Contact detection by following error (step 6a). Pure logic (no bus), so recorded runs replay.

Each cycle, every joint's position is compared with the goal it was last commanded (what the servo is
chasing). The allowed error depends on the joint's role, and must be exceeded continuously for
`contact_window_s` before it counts:
  moving   goal changed within the window    contact_err_moving   (ramp lag ~20 ticks at 150 t/s)
  holding  commanded since enable, goal fixed contact_err_holding  (gravity sag up to ~14 ticks)
  idle     not commanded since enable         contact_drift_idle   (normally 0-3 ticks)
A threshold of None disables that check. SafeArm responds by freezing (hold, torque on).

This is the industrial "following error window + time" check (CiA 402 objects 6065h/6066h) applied
per joint. Its limits (step 6 data): a servo is a spring, so gravity sag and contact look alike; the
thresholds were tuned near rest and may false-freeze under larger gravity loads until expected gravity
is subtracted (step 8). Gentle contact (wrist_roll vs shoulder, step 6) stays below every threshold:
that has to be prevented by geometry, not detected here.
"""

from dataclasses import dataclass

from so101.hw.arm import ArmSnapshot
from so101.safety.config import SafetyConfig


@dataclass(frozen=True)
class Contact:
    t_ns: int
    joint: str
    role: str  # moving | holding | idle
    error: int  # position - goal, ticks (at detection)
    threshold: int
    detail: str


class ContactChecker:
    def __init__(self, safety: SafetyConfig, hold_positions: dict[str, int], t_ns: int):
        """hold_positions: the goal every joint got at enable (its position then)."""
        self.s = safety
        self.goal = dict(hold_positions)
        self.commanded: set[str] = set()
        self.exempt: set[str] = set()  # joints not checked (e.g. the gripper while squeezing, step 10a)
        self.changed_ns: dict[str, int] = {j: t_ns - int(2 * safety.contact_window_s * 1e9) for j in hold_positions}
        self._since: dict[str, int] = {}  # joint -> t_ns the error first exceeded its threshold

    def role(self, joint: str, t_ns: int) -> str:
        if (t_ns - self.changed_ns[joint]) / 1e9 < self.s.contact_window_s:
            return "moving"
        return "holding" if joint in self.commanded else "idle"

    def threshold(self, role: str) -> int | None:
        return {"moving": self.s.contact_err_moving, "holding": self.s.contact_err_holding,
                "idle": self.s.contact_drift_idle}[role]  # fmt: skip

    def check(self, snap: ArmSnapshot) -> Contact | None:
        """Call BEFORE writing this cycle's goals: compares against the goals the servos were chasing."""
        t = snap.t_start_ns
        for joint, st in snap.joints.items():
            if st is None or joint not in self.goal or joint in self.exempt:
                self._since.pop(joint, None)
                continue
            role = self.role(joint, t)
            limit = self.threshold(role)
            err = st.position - self.goal[joint]
            if limit is None or abs(err) <= limit:
                self._since.pop(joint, None)
                continue
            start = self._since.setdefault(joint, t)
            if (t - start) / 1e9 >= self.s.contact_window_s:
                return Contact(t, joint, role, err, limit,
                               f"{role} joint {abs(err)} ticks from its goal {self.goal[joint]} "
                               f"for >= {self.s.contact_window_s} s (limit {limit})")  # fmt: skip
        return None

    def update(self, goals: dict[str, int], t_ns: int) -> None:
        """Call AFTER writing goals (only the joints actually written)."""
        for joint, g in goals.items():
            if g != self.goal.get(joint):
                self.changed_ns[joint] = t_ns
                self.goal[joint] = g
            self.commanded.add(joint)
