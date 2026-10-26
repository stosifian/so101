import json

import pytest

from so101.config import CONFIG_DIR
from so101.runlog.run import Run


def test_run_header_and_immutability(tmp_path):
    cfg = CONFIG_DIR / "arm_follower.yaml"
    run = Run("test", "follower", config_paths=[cfg], root=tmp_path)
    header = json.loads((run.dir / "header.json").read_text())
    assert header["run_id"] == run.run_id
    assert header["packages"]["feetech-servo-sdk"]
    assert "config/arm_follower.yaml" in header["config_sha256"]
    run.write_json("x.json", {})
    with pytest.raises(FileExistsError):
        run.write_json("x.json", {})
