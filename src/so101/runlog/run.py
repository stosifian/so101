"""Run directories: data/raw/<run_id>/ with a header that makes every run traceable.

A run directory is write-once: `Run` refuses to overwrite an existing file.
"""

import hashlib
import importlib.metadata
import json
import platform
import socket
import subprocess
import time
from datetime import datetime
from pathlib import Path

from so101.config import REPO_ROOT

RAW_DIR = REPO_ROOT / "data" / "raw"
TRACKED_PACKAGES = ["so101", "feetech-servo-sdk", "pyserial", "numpy", "scipy", "rerun-sdk", "pyarrow"]


def _git(*args: str) -> str | None:
    try:
        return subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=True).stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def environment_info(config_paths: list[Path] = ()) -> dict:
    versions = {}
    for pkg in TRACKED_PACKAGES:
        try:
            versions[pkg] = importlib.metadata.version(pkg)
        except importlib.metadata.PackageNotFoundError:
            versions[pkg] = None
    return {
        "git_sha": _git("rev-parse", "HEAD"),  # None until the first commit
        "git_dirty": bool(_git("status", "--porcelain")),
        "python": platform.python_version(),
        "machine": platform.machine(),
        "host": socket.gethostname(),
        "packages": versions,
        "config_sha256": {str(Path(p).relative_to(REPO_ROOT)): sha256_file(p) for p in config_paths},
    }


class Run:
    def __init__(self, kind: str, arm: str, config_paths: list[Path] = (), notes: str = "", root: Path = RAW_DIR):
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.run_id = f"{stamp}_{kind}_{arm}"
        self.dir = root / self.run_id
        self.dir.mkdir(parents=True, exist_ok=False)
        self.header = {
            "run_id": self.run_id,
            "kind": kind,
            "arm": arm,
            "started_wall": datetime.now().astimezone().isoformat(),
            "started_monotonic_ns": time.monotonic_ns(),
            "notes": notes,
            **environment_info(config_paths),
        }
        self.write_json("header.json", self.header)

    def path(self, name: str) -> Path:
        p = self.dir / name
        if p.exists():
            raise FileExistsError(f"{p} exists; raw data is immutable")
        return p

    def write_json(self, name: str, obj) -> Path:
        p = self.path(name)
        p.write_text(json.dumps(obj, indent=2))
        return p
