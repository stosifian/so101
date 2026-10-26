"""Live session plumbing shared by SafeArm scripts: hub + parquet logger (+ optional TUI / Rerun),
and a main-thread display loop around a worker thread that owns the arm.

The worker thread is the only user of the bus (SafeArm owns it); the main thread only draws.
Ctrl-C sets `stop`; the worker is expected to check it and leave its `with SafeArm(...)` block,
which turns torque off.
"""

import threading
import time
from contextlib import nullcontext

from rich.console import Console
from rich.live import Live

from so101.runlog.run import Run
from so101.telemetry.hub import Hub
from so101.telemetry.parquet_log import ParquetLogger
from so101.telemetry.tui import Tui
from so101.telemetry.viz import RerunView


class LiveSession:
    def __init__(self, run: Run, voltage: dict[str, tuple[float, float]], tui: bool, viz: bool, title: str = ""):
        self.run, self.hub, self.stop = run, Hub(), threading.Event()
        self.logger = ParquetLogger(self.hub.subscribe("parquet", maxlen=100_000), run).start()
        self.tui = Tui(self.hub.subscribe("tui", maxlen=500), voltage, title=title) if tui else None
        self.viz = (RerunView(self.hub.subscribe("viz", maxlen=5000), list(voltage), run.path("telemetry.rrd"),
                              run.run_id, run.header["started_monotonic_ns"]) if viz else None)  # fmt: skip
        self.footer: list[str] = []  # set by the caller each frame, e.g. SafeArm state

    def run_until_done(self, worker: threading.Thread, console: Console) -> bool:
        """Draw until `worker` exits. Returns True if the user pressed Ctrl-C."""
        interrupted = False
        try:
            with Live(self.tui.render(), console=console, refresh_per_second=10) if self.tui else nullcontext() as live:
                while worker.is_alive():
                    time.sleep(0.1)
                    if self.viz:
                        self.viz.update()
                    if self.tui:
                        self.tui.footer = list(self.footer)
                        self.tui.update()
                        live.update(self.tui.render())
        except KeyboardInterrupt:
            interrupted = True
            self.stop.set()
        worker.join()
        return interrupted

    def close(self) -> dict:
        summary = self.logger.stop()
        if self.viz:
            self.viz.close()
        return summary
