"""Safety settings for SafeArm, parsed and validated from config/arm_<name>.yaml."""

from dataclasses import dataclass, field

from so101.config import ArmConfig


def _threshold(section: dict, key: str) -> int | None:
    """A contact threshold: a positive number, or explicit null (= check disabled). The key must be
    present, so a missing line can't silently disable a check; 0, negatives and text are rejected."""
    if key not in section:
        raise ValueError(f"safety.{key} missing (use null to disable the check)")
    v = section[key]
    if v is None:
        return None
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0:
        raise ValueError(f"safety.{key} = {v!r}: must be a positive number of ticks, or null to disable")
    return int(v)


def _margin(section: dict, key: str) -> float | None:
    """A collision margin in metres: a number >= 0, or explicit null (= check disabled). Key must be present."""
    if key not in section:
        raise ValueError(f"safety.{key} missing (use null to disable the check)")
    v = section[key]
    if v is None:
        return None
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v < 0 or v > 0.1:
        raise ValueError(f"safety.{key} = {v!r}: must be 0..0.1 m, or null to disable")
    return float(v)


@dataclass(frozen=True)
class SafetyConfig:
    limits: dict[str, tuple[int, int]]  # joint -> (min, max) ticks, software limits
    voltage_v: tuple[float, float]
    speed_cap: int  # Goal_Speed, ticks/s
    accel_cap: int  # Acceleration register units (100 ticks/s^2)
    max_step: int  # ticks
    temp_trip_c: float
    load_trip_pct: float
    load_trip_s: float
    debounce_s: float
    comm_fail_trip: int
    torque_check_s: float
    enable_drift_tol: int  # ticks
    # contact detection (step 6a); None = check disabled (YAML null)
    contact_err_moving: int | None = None
    contact_err_holding: int | None = None
    contact_drift_idle: int | None = None
    contact_window_s: float = 0.3
    load_trip_overrides: dict[str, float] = field(default_factory=dict)  # joint -> load trip %, step 10a
    # collision model (step 8b); None = check disabled (YAML null)
    collision_table_margin_m: float | None = None
    collision_self_margin_m: float | None = None
    collision_include_camera_mount: bool = True

    def disabled_contact_checks(self) -> list[str]:
        return [n for n in ("contact_err_moving", "contact_err_holding", "contact_drift_idle") if getattr(self, n) is None]

    @classmethod
    def from_arm_config(cls, cfg: ArmConfig) -> "SafetyConfig":
        raw = cfg.raw
        if "limits_ticks" not in raw or "safety" not in raw:
            raise ValueError(f"{cfg.path}: needs limits_ticks and safety sections (follower only)")
        limits = {j: (int(lo), int(hi)) for j, (lo, hi) in raw["limits_ticks"].items()}
        if set(limits) != set(cfg.joints):
            raise ValueError(f"limits_ticks joints {sorted(limits)} != config joints {sorted(cfg.joints)}")
        for j, (lo, hi) in limits.items():
            if not 0 <= lo < hi <= 4095:
                raise ValueError(f"limits_ticks.{j} = [{lo}, {hi}] is not a valid range in 0..4095")
        s = raw["safety"]
        out = cls(
            limits=limits,
            voltage_v=tuple(cfg.expected_voltage_v),
            speed_cap=int(s["speed_cap_ticks_s"]),
            accel_cap=int(s["accel_cap"]),
            max_step=int(s["max_step_ticks"]),
            temp_trip_c=float(s["temp_trip_c"]),
            load_trip_pct=float(s["load_trip_pct"]),
            load_trip_s=float(s["load_trip_s"]),
            debounce_s=float(s["debounce_s"]),
            comm_fail_trip=int(s["comm_fail_trip"]),
            torque_check_s=float(s["torque_check_s"]),
            enable_drift_tol=int(s["enable_drift_tol_ticks"]),
            contact_err_moving=_threshold(s, "contact_err_moving_ticks"),
            contact_err_holding=_threshold(s, "contact_err_holding_ticks"),
            contact_drift_idle=_threshold(s, "contact_drift_idle_ticks"),
            contact_window_s=float(s["contact_window_s"]),
            load_trip_overrides={j: float(v) for j, v in (s.get("load_trip_pct_overrides") or {}).items()},
            collision_table_margin_m=_margin(s, "collision_table_margin_m"),
            collision_self_margin_m=_margin(s, "collision_self_margin_m"),
            collision_include_camera_mount=bool(s["collision_include_camera_mount"]),
        )
        if not 1 <= out.speed_cap <= 1000:
            raise ValueError(f"speed_cap {out.speed_cap} outside 1..1000 ticks/s (raise the bound deliberately)")
        if not 1 <= out.accel_cap <= 254:
            raise ValueError(f"accel_cap {out.accel_cap} outside 1..254")
        for j, v in out.load_trip_overrides.items():
            if j not in limits or not 0 < v <= 100:
                raise ValueError(f"load_trip_pct_overrides.{j} = {v}: unknown joint or outside 0..100 %")
        if not out.contact_window_s > 0:
            raise ValueError(f"contact_window_s {out.contact_window_s} must be > 0")
        if not 0 < out.temp_trip_c < 70:
            raise ValueError(f"temp_trip_c {out.temp_trip_c} must be below the 70 degC firmware limit")
        return out
