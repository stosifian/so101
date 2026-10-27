import pytest

from so101.hw.arm import ArmSnapshot, JointState
from so101.runlog.manifest import SCHEMA_VERSION, Manifest, read_manifest
from so101.runlog.run import Run


def snap(pos):
    return ArmSnapshot(1, 2, {"gripper": JointState(pos, 0, 12, 119, 31, 0, 0, 3)})


def test_manifest_records_poses_with_optional_camera(tmp_path):
    run = Run("test", "follower", root=tmp_path)
    m = Manifest(run, "endurance")
    m.add("ref", commanded_ticks={"gripper": 2200}, before=snap(2199), after=snap(2200),
          settle={"settled": True, "t_encoder_s": 0.21}, extra={"cycle": 0})
    m.add("ref", commanded_ticks={"gripper": 2200}, before=snap(2201), after=None)
    m.close()
    header, poses = read_manifest(run.dir / "manifest.jsonl")
    assert header["schema_version"] == SCHEMA_VERSION and header["kind"] == "endurance"
    assert len(poses) == 2 and poses[0]["camera"] is None  # Phase 2 fills this
    assert poses[0]["snapshot_after"]["joints"]["gripper"]["position"] == 2200
    assert poses[1]["snapshot_after"] is None


def test_manifest_file_is_write_once(tmp_path):
    run = Run("test", "follower", root=tmp_path)
    Manifest(run, "x").close()
    with pytest.raises(FileExistsError):
        Manifest(run, "x")
