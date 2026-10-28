import pytest

from so101.sensors import MockCamera, Sensor
from so101.sensors.camera import parse_config_value, parse_summary

SUMMARY = """Camera summary:
Manufacturer: Sony Corporation
Model: ILCE-7M4
  Version: 1.00
  Serial Number: 00000000000000003
Device Version: 3.01
Model: should-not-override
"""


def test_parse_summary():
    s = parse_summary(SUMMARY)
    assert s == {"manufacturer": "Sony Corporation", "model": "ILCE-7M4", "ptp_version": "1.00",
                 "serial": "00000000000000003", "firmware": "3.01"}  # fmt: skip


def test_parse_config_value():
    txt = "Label: F-Number\nReadonly: 0\nType: RADIO\nCurrent: 11\nChoice: 0 1.8\n"
    assert parse_config_value(txt) == "11"
    assert parse_config_value("nothing here") is None


def test_mock_camera_writes_files_and_times(tmp_path):
    cam = MockCamera(delay_s=0.01)
    assert isinstance(cam, Sensor)
    r = cam.capture(tmp_path, "img_0000")
    assert r.ok and r.files == [tmp_path / "img_0000.mock"] and r.duration_s >= 0.01
    assert r.as_dict()["files"] == [str(tmp_path / "img_0000.mock")]


def test_mock_camera_injected_failure(tmp_path):
    cam = MockCamera(fail_at={1})
    assert cam.capture(tmp_path, "a").ok
    r = cam.capture(tmp_path, "b")
    assert not r.ok and r.files == [] and r.error == "injected failure"


def test_capture_never_overwrites(tmp_path):
    cam = MockCamera()
    cam.capture(tmp_path, "img")
    with pytest.raises(FileExistsError):
        cam.capture(tmp_path, "img")
