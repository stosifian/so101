"""Feetech STS3215 control table — the single source of truth for register addresses.

Cross-checked against LeRobot's SCS_SERIES_CONTROL_TABLE (shoggoth/lerobot @ b536f47e)
plus the firmware-version bytes at 0–1. Values on the bus are little-endian.

Some registers are *sign-magnitude*, not two's complement: bit `sign_bit` is the sign and
the bits below it are the magnitude (e.g. Present_Load uses bit 10). `decode()` handles that.
"""

from dataclasses import dataclass
from typing import Literal

Area = Literal["eeprom", "sram"]


@dataclass(frozen=True)
class Register:
    name: str
    addr: int
    size: int  # bytes: 1 or 2
    area: Area
    sign_bit: int | None = None  # sign-magnitude encoding if set
    unit: str = ""  # human unit of the decoded value, for docs and dumps


def _r(name, addr, size, area, sign_bit=None, unit=""):
    return Register(name, addr, size, area, sign_bit, unit)


# fmt: off
REGISTERS: dict[str, Register] = {r.name: r for r in [
    # --- EEPROM (persisted; writes need Lock=0 and wear the flash — we never write these casually)
    _r("Firmware_Major",        0, 1, "eeprom"),
    _r("Firmware_Minor",        1, 1, "eeprom"),
    _r("Model",                 3, 2, "eeprom"),
    _r("ID",                    5, 1, "eeprom"),
    _r("Baud_Rate",             6, 1, "eeprom", unit="index (0=1M)"),
    _r("Return_Delay",          7, 1, "eeprom", unit="2 us"),
    _r("Response_Status_Level", 8, 1, "eeprom"),
    _r("Min_Angle_Limit",       9, 2, "eeprom", unit="ticks"),
    _r("Max_Angle_Limit",      11, 2, "eeprom", unit="ticks"),
    _r("Max_Temperature_Limit",13, 1, "eeprom", unit="degC"),
    _r("Max_Voltage_Limit",    14, 1, "eeprom", unit="0.1 V"),
    _r("Min_Voltage_Limit",    15, 1, "eeprom", unit="0.1 V"),
    _r("Max_Torque_Limit",     16, 2, "eeprom", unit="0.1 %"),
    _r("Phase",                18, 1, "eeprom"),
    _r("Unloading_Condition",  19, 1, "eeprom", unit="bitmask"),
    _r("LED_Alarm_Condition",  20, 1, "eeprom", unit="bitmask"),
    _r("P_Coefficient",        21, 1, "eeprom"),
    _r("D_Coefficient",        22, 1, "eeprom"),
    _r("I_Coefficient",        23, 1, "eeprom"),
    _r("Minimum_Startup_Force",24, 2, "eeprom", unit="0.1 %"),
    _r("CW_Dead_Zone",         26, 1, "eeprom", unit="ticks"),
    _r("CCW_Dead_Zone",        27, 1, "eeprom", unit="ticks"),
    _r("Protection_Current",   28, 2, "eeprom", unit="6.5 mA"),
    _r("Angular_Resolution",   30, 1, "eeprom"),
    _r("Offset",               31, 2, "eeprom", sign_bit=11, unit="ticks"),
    _r("Mode",                 33, 1, "eeprom", unit="0=pos 1=vel 2=pwm 3=step"),
    _r("Protective_Torque",    34, 1, "eeprom", unit="%"),
    _r("Protection_Time",      35, 1, "eeprom", unit="10 ms"),
    _r("Overload_Torque",      36, 1, "eeprom", unit="%"),
    _r("Velocity_P",           37, 1, "eeprom"),
    _r("Over_Current_Protection_Time", 38, 1, "eeprom", unit="10 ms"),
    _r("Velocity_I",           39, 1, "eeprom"),
    # --- SRAM (volatile; reset on power cycle)
    _r("Torque_Enable",        40, 1, "sram"),
    _r("Acceleration",         41, 1, "sram", unit="100 ticks/s^2"),
    _r("Goal_Position",        42, 2, "sram", sign_bit=15, unit="ticks"),
    _r("Goal_Time",            44, 2, "sram", unit="ms"),
    _r("Goal_Speed",           46, 2, "sram", sign_bit=15, unit="ticks/s"),
    _r("Torque_Limit",         48, 2, "sram", unit="0.1 %"),
    _r("Lock",                 55, 1, "sram"),
    _r("Present_Position",     56, 2, "sram", sign_bit=15, unit="ticks"),
    _r("Present_Speed",        58, 2, "sram", sign_bit=15, unit="ticks/s"),
    _r("Present_Load",         60, 2, "sram", sign_bit=10, unit="0.1 %"),
    _r("Present_Voltage",      62, 1, "sram", unit="0.1 V"),
    _r("Present_Temperature",  63, 1, "sram", unit="degC"),
    _r("Status",               65, 1, "sram", unit="bitmask"),
    _r("Moving",               66, 1, "sram"),
    _r("Present_Current",      69, 2, "sram", sign_bit=15, unit="6.5 mA"),
]}
# fmt: on


def decode(reg: Register, raw: int) -> int:
    """Raw unsigned register value -> signed integer (sign-magnitude aware)."""
    if reg.sign_bit is None:
        return raw
    magnitude = raw & ((1 << reg.sign_bit) - 1)
    return -magnitude if raw & (1 << reg.sign_bit) else magnitude


def encode(reg: Register, value: int) -> int:
    """Signed integer -> raw unsigned register value. Inverse of decode()."""
    if reg.sign_bit is None:
        if not 0 <= value < (1 << (8 * reg.size)):
            raise ValueError(f"{reg.name}: {value} out of range")
        return value
    limit = 1 << reg.sign_bit
    if abs(value) >= limit:
        raise ValueError(f"{reg.name}: |{value}| exceeds {limit - 1}")
    return (abs(value) | limit) if value < 0 else value
