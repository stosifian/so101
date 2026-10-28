"""Sensors behind small interfaces so backends are swappable (Phase 2: camera; Phase 3: IMU, wrist camera)."""

from so101.sensors.base import Sensor
from so101.sensors.camera import Camera, CaptureResult, Gphoto2Camera, MockCamera

__all__ = ["Sensor", "Camera", "CaptureResult", "Gphoto2Camera", "MockCamera"]
