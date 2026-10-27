"""Settling and approach direction (step 6c; Phase 2 obligation).

Measured behaviour this is built on (docs/hardware_log.md):
- `Moving` = "servo still executing its move": it drops to 0 when the servo considers itself done,
  while the joint can still sit 1-3 ticks short (unloaded) or ~20 short (shoulder_lift going up).
- So "settled" here means *stopped*: Moving == 0 and position steady (within `steady_ticks`) for
  `stable_s`. The remaining error to goal is a measured result, not a pass/fail, unless `tol` is given.
- The error depends on approach direction (hysteresis 2-20 ticks by joint), so metrology moves should
  approach every target from the same side: `approach()` overshoots to the far side first.
- The encoders don't see structural wobble (links, mounts, clamp: user saw end-effector wobble at stops
  while encoders showed no ringing). `dwell_s` adds a fixed wait after encoder settle. Its value is a
  placeholder until ring-down is measured (camera in Phase 2).
LeRobot has no equivalent (its loop only streams goals), so this is ours.
"""

import time
from dataclasses import dataclass, field

from so101.motion.loop import Segment, control_loop, sequence
from so101.safety.safearm import SafeArm

DEFAULT_DWELL_S = 0.5  # placeholder: structural ring-down not yet measured


@dataclass
class SettleResult:
    settled: bool
    t_encoder_s: float | None  # time until encoder criteria held (Moving 0 + steady), from call
    t_total_s: float  # including dwell
    errors: dict[str, int] = field(default_factory=dict)  # position - last goal at the end
    positions: dict[str, int] = field(default_factory=dict)
    not_settled: list[str] = field(default_factory=list)  # joints that failed at timeout
    approach_dir: dict[str, int] = field(default_factory=dict)


def wait_until_settled(arm: SafeArm, joints: list[str] | None = None, *, tol: int | None = None,
                       steady_ticks: int = 1, stable_s: float = 0.2, dwell_s: float = DEFAULT_DWELL_S,
                       timeout_s: float = 3.0, rate_hz: float = 100.0,
                       approach_dir: dict[str, int] | None = None) -> SettleResult:  # fmt: skip
    """Hold (no new goals, all safety checks running) until every joint in `joints` has Moving == 0 and
    has moved at most `steady_ticks` over the last `stable_s` (and, if `tol` is set, is within `tol` of
    its last goal); then wait `dwell_s` more. Trips and contact freezes still apply while waiting."""
    joints = joints or list(arm.cfg.joints)
    period = 1.0 / rate_hz
    t0 = time.monotonic()
    history: dict[str, list[tuple[float, int]]] = {j: [] for j in joints}
    t_encoder = None
    last = None
    while True:
        now = time.monotonic() - t0
        last = arm.step().snapshot
        goals = arm.last_goals
        ok: dict[str, bool] = {}
        for j in joints:
            st = last.joints.get(j)
            if st is None:
                ok[j] = False
                continue
            h = history[j]
            h.append((now, st.position))
            while h and now - h[0][0] > stable_s:
                h.pop(0)
            window_full = now >= stable_s and h[0][0] <= now - stable_s * 0.9
            steady = window_full and max(p for _, p in h) - min(p for _, p in h) <= steady_ticks
            in_tol = tol is None or (j in goals and abs(st.position - goals[j]) <= tol)
            ok[j] = st.moving == 0 and steady and in_tol
        if all(ok.values()):
            t_encoder = now if t_encoder is None else t_encoder
        else:
            t_encoder = None  # must hold continuously through the dwell
        if t_encoder is not None and now - t_encoder >= dwell_s:
            break
        if now >= timeout_s:
            break
        time.sleep(period)
    goals = arm.last_goals
    errors = {j: last.joints[j].position - goals[j] for j in joints if last.joints.get(j) and j in goals}
    positions = {j: last.joints[j].position for j in joints if last.joints.get(j)}
    settled = t_encoder is not None and now - t_encoder >= dwell_s
    return SettleResult(settled, t_encoder, now, errors, positions, [j for j, good in ok.items() if not good],
                        dict(approach_dir or {}))


def approach(arm: SafeArm, targets: dict[str, float], approach_dir: dict[str, int], *, speed: float = 150.0,
             overshoot: int = 40, easing: str = "minjerk", rate_hz: float = 100.0, settle: bool = True,
             stop=None, freeze=None, **settle_kw) -> SettleResult | None:  # fmt: skip
    """Move each joint to its target so that the LAST motion is in `approach_dir[j]` (+1 = increasing ticks):
    if the joint would otherwise arrive from the wrong side, it first goes `overshoot` ticks past the target
    on the opposite side. Then (optionally) wait_until_settled. Uses the normal SafeArm clamps and checks."""
    snap = arm.step().snapshot
    start = {j: float(snap.joints[j].position) for j in targets}
    pre = {}
    for j, tgt in targets.items():
        d = approach_dir[j]
        if d not in (1, -1):
            raise ValueError(f"approach_dir[{j}] must be +1 or -1")
        # arriving direction is +1 if we come from below: need start < target; else go below first
        pre[j] = tgt - d * overshoot if (tgt - start[j]) * d <= overshoot else start[j]
    segs = [Segment(start, pre, speed, easing), Segment(pre, dict(targets), speed, easing)]
    goal_fn, total = sequence(segs)
    control_loop(arm, rate_hz, goal_fn, total + 0.05, stop, freeze=freeze)
    if arm.frozen or (stop is not None and stop.is_set()):
        return None
    return wait_until_settled(arm, list(targets), rate_hz=rate_hz, approach_dir=approach_dir, **settle_kw) if settle else None
