# Lessons learned

Gotchas we hit, with the root cause rather than just the workaround. Each entry has:
**Symptom** / **What was going on** / **Why it happens** / **How we found it** / **The fix** / **Takeaway**.

---

## 1. The arm whose board enumerates but whose servos are silent

*Logged 2026-10-26, Step 1 (port discovery).*

**Symptom.** The leader's USB port appeared (`/dev/cu.usbmodem5B901013071`), but every ping to IDs 1–6 returned "There is no status packet!".
**What was going on.** The driver board was not connected to the servo daisy-chain.
**Why it happens.** USB powers and enumerates only the board's USB-serial bridge (CH343). The servo bus is a separate physical link with its own supply. A missing first link (board → servo 1) silences the whole chain, because the servos are daisy-chained.
**How we found it.** A voltage probe identified the follower (11.9 V). The leader was silent. A ping scan of IDs 0–20/253 at 8 baud rates was empty, which ruled out baud/ID config, so the fault had to be physical.
**The fix.** Reconnect the driver→motor cable. All six answered at 4.9 V.
**Takeaway.** "Port exists" ≠ "servos reachable". If a ping scan across all bauds is empty, stop debugging software and check the board→servo cable, servo power and the channel jumper.

## 2. The editable install that iCloud quietly broke

*Logged 2026-10-26, Step 2 (first pytest run). Likely also the real root cause of shoggoth lesson #3.*

**Symptom.** `import so101` worked right after `uv sync`, but a few minutes later `uv run pytest` and `uv run python -c "import so101"` both failed with `ModuleNotFoundError`. `pip`/uv listed the package as installed.
**What was going on.** The editable install puts a one-line `_editable_impl_so101.pth` in site-packages pointing at `src/`. That file had the macOS `hidden` file flag set (`ls -lO` shows `hidden`), as did 67 of 68 entries in site-packages.
**Why it happens.** Two separately reasonable behaviours collide:
- `~/Desktop` is synced by **iCloud Drive ("Desktop & Documents")**, which sets `UF_HIDDEN` on files inside dot-folders like `.venv`, about 1 s after creation.
- **Python 3.12's `site.py` skips hidden `.pth` files** (a security hardening; `site.py` line ~176: "Skipping hidden .pth file"). The path to `src/` therefore never reaches `sys.path`.
The import worked at first only because it ran before iCloud reached the new files.
**How we found it.** `sys.path` lacked `src/` even though the `.pth` existed and had the right content, so the `.pth` was being ignored. `ls -lO` showed `hidden`. `chflags nohidden` fixed the import until the next run re-hid it. A bare `touch .hidtest/a.pth` turned hidden within ~1 s, which pinned it on iCloud rather than uv.
**The fix.** Moved the repo out of iCloud to `~/dev/so101`. That fixes the venv, and it also keeps Phase 2 RAW data off iCloud and keeps `.git` from being synced file by file.
**Takeaway.** Don't keep Python projects (or any `.venv`) under iCloud-synced `~/Desktop` or `~/Documents`. If an installed package "isn't there" on macOS, check `ls -lO` on its `.pth` for the `hidden` flag.
