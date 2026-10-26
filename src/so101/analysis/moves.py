"""Analysis of commanded moves from a run's telemetry rows (parquet, one row per joint per sample).

Shared by scripts/motion/single_joint_move.py (step 5c/5d) and joint_sweep.py (step 6).
"""

from statistics import median

SETTLE_TOL = 2  # ticks
STICK_TOL = 4  # ticks; above the 1-3 tick steady-state error (step 5c), so holds don't count as stalls
TAIL_S = 0.5  # steady-state numbers use the last 0.5 s of each hold


def analyse(rows: list[dict], joint: str, min_hold_samples: int = 30) -> dict:
    """Group the joint's samples into runs of identical commanded goal. A run of >= min_hold_samples
    (0.3 s at 100 Hz) is a hold; shorter runs are part of a streamed ramp (at 50 ticks/s the rounded
    goal repeats every other cycle). Every hold after the first is a stop at a new goal: in stream
    mode it follows a ramp, in servo mode the goal stepped there and the servo is still travelling.

    Stick-slip: among consecutive samples where the joint is > STICK_TOL from its goal (so it should
    be moving, not just sitting at its steady-state error), the fraction where the position did not
    change, and the longest such stall."""
    js = sorted((r for r in rows if r["joint"] == joint and r["goal"] is not None and r["position"] is not None),
                key=lambda r: r["seq"])  # fmt: skip
    runs: list[list[dict]] = []
    for r in js:
        if runs and runs[-1][0]["goal"] == r["goal"]:
            runs[-1].append(r)
        else:
            runs.append([r])
    ramps, holds, stops = [], [], []
    for k, run_ in enumerate(runs):
        if len(run_) < min_hold_samples:
            ramps += run_
            continue
        holds += run_
        if k == 0:
            continue
        target = run_[0]["goal"]
        direction = 1 if target > runs[k - 1][0]["goal"] else -1
        t_stop = run_[0]["t_start_ns"]
        over = max(direction * (r["position"] - target) for r in run_)
        settled_at = next(((run_[i]["t_start_ns"] - t_stop) / 1e6 for i in range(len(run_))
                           if all(abs(r["position"] - target) <= SETTLE_TOL for r in run_[i:])), None)  # fmt: skip
        last_moving = max((i for i, r in enumerate(run_) if r["moving"]), default=-1)
        moving_off = (run_[last_moving + 1]["t_start_ns"] - t_stop) / 1e6 if last_moving + 1 < len(run_) else None
        tail = [r for r in run_ if (run_[-1]["t_start_ns"] - r["t_start_ns"]) / 1e9 <= TAIL_S]
        stops.append({"target": target, "direction": direction, "overshoot_ticks": max(over, 0),
                      "settle_ms": settled_at, "moving_zero_after_ms": moving_off,
                      "final_error": run_[-1]["position"] - target,
                      "tail_error_median": median(r["position"] - target for r in tail),
                      "tail_jitter_span": max(r["position"] for r in tail) - min(r["position"] for r in tail),
                      "tail_load_median_pct": median(r["load"] for r in tail) / 10,
                      "hold_s": (run_[-1]["t_start_ns"] - t_stop) / 1e9})  # fmt: skip
    err = [abs(r["goal"] - r["position"]) for r in ramps]
    should_move = [(a, b) for a, b in zip(js, js[1:]) if abs(b["goal"] - a["position"]) > STICK_TOL]
    stuck_flags = [b["position"] == a["position"] for a, b in should_move]
    longest = cur = 0
    for (a, b), stuck in zip(should_move, stuck_flags):
        cur = cur + (b["t_start_ns"] - a["t_start_ns"]) if stuck else 0
        longest = max(longest, cur)
    return {
        "ramp_samples": len(ramps),
        "hold_samples": len(holds),
        "ramp_err_median": median(err) if err else None,
        "ramp_err_max": max(err) if err else None,
        "moving_frac_ramp": sum(r["moving"] for r in ramps) / len(ramps) if ramps else None,
        "moving_frac_hold": sum(r["moving"] for r in holds) / len(holds) if holds else None,
        "load_med_ramp_pct": median(abs(r["load"]) for r in ramps) / 10 if ramps else None,
        "load_med_hold_pct": median(abs(r["load"]) for r in holds) / 10 if holds else None,
        "current_max_units": max((r["current"] for r in js), default=None),
        "stuck_frac_while_off_goal": sum(stuck_flags) / len(stuck_flags) if stuck_flags else None,
        "longest_stall_ms": longest / 1e6,
        "stops": stops,
    }


def hysteresis(stops: list[dict]) -> list[dict]:
    """Per target visited from both directions: steady-state error approaching upward vs downward,
    and holding load (gravity profile). The difference is the backlash/approach-direction effect."""
    by_target: dict[int, dict[int, list[dict]]] = {}
    for s in stops:
        by_target.setdefault(s["target"], {}).setdefault(s["direction"], []).append(s)
    out = []
    for target in sorted(by_target):
        up, down = by_target[target].get(1, []), by_target[target].get(-1, [])
        e_up = median(s["tail_error_median"] for s in up) if up else None
        e_dn = median(s["tail_error_median"] for s in down) if down else None
        out.append({
            "target": target,
            "err_up": e_up, "err_down": e_dn,
            "hysteresis": None if e_up is None or e_dn is None else e_dn - e_up,
            "load_up_pct": median(s["tail_load_median_pct"] for s in up) if up else None,
            "load_down_pct": median(s["tail_load_median_pct"] for s in down) if down else None,
            "jitter_max": max(s["tail_jitter_span"] for s in up + down),
        })  # fmt: skip
    return out
