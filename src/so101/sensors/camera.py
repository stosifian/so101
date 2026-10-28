"""Still camera for quasi-static metrology (Phase 2 step 0; spec §6).

`Camera.capture(dest_dir, stem)` triggers one exposure and returns once the file(s) are on the host, with
monotonic timestamps at request and completion. Backends:
  - `Gphoto2Camera`: the gphoto2 CLI (brew install gphoto2). A subprocess per capture, so a hung camera
    is a timeout here, not a frozen script; that is what step 1's 50-capture test measures.
  - `MockCamera`: writes a placeholder file; optional delay and injected failures. Dry runs and tests.
The Sony Camera Remote SDK is the fallback (spec §6) if gphoto2 fails step 1; it goes behind this interface.
No lens corrections are applied anywhere here: files are stored as the camera delivers them (RAW expected).
"""

import shutil
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

# gphoto2 config entries recorded in the run header if the camera exposes them (names vary by model; missing = skipped)
CONFIG_KEYS = ["f-number", "shutterspeed", "iso", "whitebalance", "imagequality", "imageformat", "focusmode",
               "exposurecompensation", "capturetarget", "batterylevel"]  # fmt: skip


@dataclass
class CaptureResult:
    ok: bool
    files: list[Path]
    t_request_ns: int
    t_done_ns: int
    error: str | None = None
    stdout: str = ""

    @property
    def duration_s(self) -> float:
        return (self.t_done_ns - self.t_request_ns) / 1e9

    def as_dict(self) -> dict:
        return {"ok": self.ok, "files": [str(f) for f in self.files], "t_request_ns": self.t_request_ns,
                "t_done_ns": self.t_done_ns, "duration_s": self.duration_s, "error": self.error}  # fmt: skip


class Camera(Protocol):
    name: str

    def info(self) -> dict: ...

    def capture(self, dest_dir: Path, stem: str) -> CaptureResult: ...

    def close(self) -> None: ...


def parse_summary(text: str) -> dict:
    """Pick model / firmware / serial out of `gphoto2 --summary`."""
    keys = {"Model": "model", "Manufacturer": "manufacturer", "Device Version": "firmware",
            "Serial Number": "serial", "Version": "ptp_version"}  # fmt: skip
    out = {}
    for line in text.splitlines():
        k, sep, v = line.strip().partition(":")
        if sep and k.strip() in keys and keys[k.strip()] not in out:
            out[keys[k.strip()]] = v.strip()
    return out


def parse_config_value(text: str) -> str | None:
    """`gphoto2 --get-config X` → its 'Current:' value."""
    for line in text.splitlines():
        if line.startswith("Current:"):
            return line.partition(":")[2].strip()
    return None


def _no_overwrite(dest_dir: Path, stem: str) -> None:
    if any(dest_dir.glob(f"{stem}.*")):
        raise FileExistsError(f"{dest_dir}/{stem}.* exists; raw data is immutable")


class Gphoto2Camera:
    name = "gphoto2"

    def __init__(self, timeout_s: float = 30.0, binary: str = "gphoto2"):
        self.binary = shutil.which(binary)
        if self.binary is None:
            raise FileNotFoundError("gphoto2 not found: brew install gphoto2")
        self.timeout_s = timeout_s

    def _run(self, *args: str, timeout: float | None = None) -> subprocess.CompletedProcess:
        return subprocess.run([self.binary, *args], capture_output=True, text=True, timeout=timeout or self.timeout_s)

    def detect(self) -> str:
        return self._run("--auto-detect", timeout=10).stdout

    def info(self) -> dict:
        r = self._run("--summary", timeout=15)
        if r.returncode != 0:
            raise RuntimeError(f"gphoto2 --summary failed: {r.stderr.strip() or r.stdout.strip()}{hint(r.stderr)}")
        info = {"backend": self.name, **parse_summary(r.stdout),
                "gphoto2": self._run("--version", timeout=10).stdout.splitlines()[0:1]}  # fmt: skip
        config = {}
        for k in CONFIG_KEYS:
            c = self._run("--get-config", k, timeout=10)
            if c.returncode == 0:
                config[k] = parse_config_value(c.stdout)
        info["config"] = config
        return info

    def capture(self, dest_dir: Path, stem: str) -> CaptureResult:
        dest_dir.mkdir(parents=True, exist_ok=True)
        _no_overwrite(dest_dir, stem)
        t0 = time.monotonic_ns()
        try:
            r = self._run("--capture-image-and-download", "--filename", str(dest_dir / f"{stem}.%C"))
        except subprocess.TimeoutExpired:
            return CaptureResult(False, sorted(dest_dir.glob(f"{stem}.*")), t0, time.monotonic_ns(),
                                 f"timeout after {self.timeout_s:g} s (hang)")  # fmt: skip
        t1 = time.monotonic_ns()
        files = sorted(dest_dir.glob(f"{stem}.*"))
        if r.returncode != 0:
            return CaptureResult(False, files, t0, t1, f"exit {r.returncode}: {r.stderr.strip()[-300:]}{hint(r.stderr)}", r.stdout)
        if not files:
            return CaptureResult(False, files, t0, t1, "gphoto2 exited 0 but no file was downloaded", r.stdout)
        return CaptureResult(True, files, t0, t1, None, r.stdout)

    def close(self) -> None:
        pass  # one process per call: nothing held open


def hint(stderr: str) -> str:
    if "Could not claim the USB device" in stderr or "-53" in stderr:
        return " [hint: macOS's ptpcamerad holds the camera: `killall ptpcamerad`, then retry]"
    if "No camera found" in stderr or "-105" in stderr:
        return " [hint: camera off/asleep, or USB Connection is not 'PC Remote']"
    return ""


@dataclass
class MockCamera:
    """Writes `<stem>.mock` (a few bytes). `fail_at` = capture indices (0-based) that fail."""

    delay_s: float = 0.0
    fail_at: set[int] = field(default_factory=set)
    name: str = "mock"
    count: int = 0

    def info(self) -> dict:
        return {"backend": self.name, "model": "MockCamera", "firmware": "0", "config": {}}

    def capture(self, dest_dir: Path, stem: str) -> CaptureResult:
        dest_dir.mkdir(parents=True, exist_ok=True)
        _no_overwrite(dest_dir, stem)
        i, self.count = self.count, self.count + 1
        t0 = time.monotonic_ns()
        time.sleep(self.delay_s)
        if i in self.fail_at:
            return CaptureResult(False, [], t0, time.monotonic_ns(), "injected failure")
        f = dest_dir / f"{stem}.mock"
        f.write_bytes(f"mock capture {i}\n".encode())
        return CaptureResult(True, [f], t0, time.monotonic_ns())

    def close(self) -> None:
        pass
