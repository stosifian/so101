"""The `Sensor` interface: anything that measures the arm from outside the servo bus."""

from typing import Protocol, runtime_checkable


@runtime_checkable
class Sensor(Protocol):
    name: str

    def info(self) -> dict:
        """Identity + settings for the run header (model, serial, firmware, ...)."""
        ...

    def close(self) -> None: ...
