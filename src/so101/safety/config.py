"""Safety settings for SafeArm, parsed and validated from config/arm_<name>.yaml."""

from dataclasses import dataclass

from so101.config import ArmConfig


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
        )
        if not 1 <= out.speed_cap <= 1000:
            raise ValueError(f"speed_cap {out.speed_cap} outside 1..1000 ticks/s (raise the bound deliberately)")
        if not 1 <= out.accel_cap <= 254:
            raise ValueError(f"accel_cap {out.accel_cap} outside 1..254")
        if not 0 < out.temp_trip_c < 70:
            raise ValueError(f"temp_trip_c {out.temp_trip_c} must be below the 70 degC firmware limit")
        return out
