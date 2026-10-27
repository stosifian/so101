"""Live session plumbing shared by SafeArm scripts: hub + parquet logger (+ optional TUI / Rerun),
and a main-thread display loop around a worker thread that owns the arm.

The worker thread is the only user of the bus (SafeArm owns it); the main thread only draws.
Two-stage Ctrl-C (freeze_first=True, the default):
  1st Ctrl-C sets `freeze`: the worker's control_loop calls SafeArm.freeze() (goal := present, torque
     stays on, nothing falls) and keeps monitoring.
  2nd Ctrl-C sets `stop`: the worker leaves its `with SafeArm(...)` block, which turns torque off.
With freeze_first=False the first Ctrl-C sets `stop` directly.
The terminal must have keyboard focus: Ctrl-C in the Rerun window does nothing.
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
    def __init__(self, run: Run, voltage: dict[str, tuple[float, float]], tui: bool, viz: bool, title: str = "",
                 freeze_first: bool = True):  # fmt: skip
        self.run, self.hub, self.stop, self.freeze = run, Hub(), threading.Event(), threading.Event()
        self.freeze_first = freeze_first
        self.logger = ParquetLogger(self.hub.subscribe("parquet", maxlen=100_000), run).start()
        self.tui = Tui(self.hub.subscribe("tui", maxlen=500), voltage, title=title) if tui else None
        self.viz = (RerunView(self.hub.subscribe("viz", maxlen=5000), list(voltage), run.path("telemetry.rrd"),
                              run.run_id, run.header["started_monotonic_ns"]) if viz else None)  # fmt: skip
        self._viz_all = self.viz  # kept for close(), even if the live display is dropped
        self.footer: list[str] = []  # set by the caller, e.g. what's running
        self.status_fn = None  # optional () -> str | None, polled every frame (e.g. "FROZEN: contact ...")
        self._status_shown = None

    def run_until_done(self, worker: threading.Thread, console: Console) -> bool:
        """Draw until `worker` exits. Returns True if the user pressed Ctrl-C at least once.
        A display error (e.g. the Rerun viewer closed by Ctrl-C) disables that display; it never
        stops this loop, because the worker may be holding the arm and waiting for the 2nd Ctrl-C."""
        interrupts = 0
        with Live(self.tui.render(), console=console, refresh_per_second=10) if self.tui else nullcontext() as live:
            while worker.is_alive():
                try:
                    time.sleep(0.1)
                    if self.viz:
                        try:
                            self.viz.update()
                        except Exception as e:  # viewer gone; keep the session alive
                            console.print(f"[yellow]Rerun display stopped: {e}")
                            self.viz = None
                    status = self.status_fn() if self.status_fn else None
                    if status and status != self._status_shown and not self.tui:
                        console.print(f"[bold yellow]{status}")
                    self._status_shown = status
                    if self.tui:
                        self.tui.footer = list(self.footer) + ([status] if status else [])
                        self.tui.update()
                        live.update(self.tui.render())
                except KeyboardInterrupt:
                    interrupts += 1
                    if interrupts == 1 and self.freeze_first:
                        self.freeze.set()
                        self.footer = ["FROZEN: holding position, torque ON. Support the arm, then Ctrl-C again = torque off."]
                        if not self.tui:
                            console.print(f"[bold yellow]{self.footer[0]}")
                    else:
                        self.stop.set()
        worker.join()
        return interrupts > 0

    def close(self) -> dict:
        summary = self.logger.stop()
        if self._viz_all:
            self._viz_all.close()  # best effort; still writes the .rrd file sink
        return summary
