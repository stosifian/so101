"""Load per-arm hardware config (config/arm_<name>.yaml)."""

from dataclasses import dataclass
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO_ROOT / "config"


@dataclass(frozen=True)
class ArmConfig:
    name: str
    port: str
    baudrate: int
    protocol_end: int
    joints: dict[str, int]  # joint name -> servo id, in kinematic order
    expected_voltage_v: tuple[float, float]
    path: Path
    raw: dict  # full yaml, so later steps (limits, speed caps) can add keys without changing this class

    @property
    def ids(self) -> list[int]:
        return list(self.joints.values())


def load_arm_config(name: str, config_dir: Path = CONFIG_DIR) -> ArmConfig:
    path = config_dir / f"arm_{name}.yaml"
    data = yaml.safe_load(path.read_text())
    return ArmConfig(
        name=data["name"],
        port=data["port"],
        baudrate=data["baudrate"],
        protocol_end=data["protocol_end"],
        joints=dict(data["joints"]),
        expected_voltage_v=tuple(data["expected_voltage_v"]),
        path=path,
        raw=data,
    )
