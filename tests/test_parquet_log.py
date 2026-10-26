import json
import time

import pyarrow.parquet as pq

from so101.hw.arm import Arm
from so101.hw.bus import MockBus
from so101.runlog.run import Run
from so101.telemetry.hub import Hub
from so101.telemetry.parquet_log import ParquetLogger
from so101.telemetry.poller import Poller

JOINTS = {"shoulder_pan": 1, "gripper": 6}


def test_logger_writes_every_sample_as_long_rows(tmp_path):
    run = Run("test", "follower", root=tmp_path)
    hub = Hub()
    logger = ParquetLogger(hub.subscribe("parquet", maxlen=100_000), run, flush_s=0.1).start()
    with Poller(Arm(MockBus.with_servos([1, 6], position=1234), JOINTS), hub, "follower", rate_hz=200) as p:
        time.sleep(0.35)
    summary = logger.stop()

    t = pq.read_table(run.dir / "telemetry")
    assert summary["parts"] >= 2  # flushed more than once
    assert summary["samples"] == p.stats.cycles
    assert t.num_rows == summary["rows"] == 2 * p.stats.cycles
    assert set(t["joint"].to_pylist()) == set(JOINTS)
    assert set(t["position"].to_pylist()) == {1234}
    seqs = sorted(set(t["seq"].to_pylist()))
    assert seqs == list(range(p.stats.cycles))  # nothing lost between poller and disk
    assert json.loads((run.dir / "telemetry_summary.json").read_text())["dropped_by_hub"] == 0


def test_failed_read_logs_nulls_and_error_text(tmp_path):
    run = Run("test", "follower", root=tmp_path)
    hub = Hub()
    logger = ParquetLogger(hub.subscribe("parquet"), run, flush_s=0.1).start()
    with Poller(Arm(MockBus.with_servos([1]), JOINTS), hub, "follower", rate_hz=100):  # gripper missing
        time.sleep(0.05)
    logger.stop()

    rows = pq.read_table(run.dir / "telemetry").to_pylist()
    grip = [r for r in rows if r["joint"] == "gripper"]
    assert grip and all(r["position"] is None and "no status packet" in r["error"] for r in grip)
    pan = [r for r in rows if r["joint"] == "shoulder_pan"]
    assert all(r["error"] is None and r["position"] == 2048 for r in pan)
