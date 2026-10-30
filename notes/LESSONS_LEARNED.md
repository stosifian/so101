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

## 3. The camera that "couldn't be claimed"

*Logged 2026-10-28, Phase 2 (tethering).*

**Symptom.** `gphoto2 --summary` failed with "Could not claim the USB device (-53)", although the same setup had captured 50/50 earlier.
**What was going on.** macOS's `ptpcamerad` had taken the camera. Photos.app was open (it had been used to AirDrop the calibration pattern), and it keeps that daemon alive.
**Why it happens.** Only one process can own a PTP device. macOS starts `ptpcamerad` for Photos/Image Capture whenever a camera appears, and it wins the race.
**How we found it.** `pgrep -lf "ptpcamerad|Photos"` listed both.
**The fix.** Quit Photos, `killall ptpcamerad`, retry (replug if it comes back).
**Takeaway.** Before tethering: no Photos, Image Capture or Preview open. The capture code prints this hint on error -53.
**Recurrence (2026-10-29).** Same error, but this time Preview was open (used for phone photos). While Preview held it, `ptpcamerad` ignored `killall`: the same PID survived for hours. Quitting Preview fixed it. Check `pgrep -lf "ptpcamerad|Photos|Image Capture|Preview"`; a PID that survives `killall` means an app still holds it.

## 4. Drift blamed on the camera that was the target moving

*Logged 2026-10-28, Phase 2 (stability check).*

**Symptom.** A static iPad board drifted 0.6 px over a minute, with a 52″ in-plane rotation.
**What was going on.** The iPad moved within the scene (stepwise, back and forth). The camera was steady.
**Why it happens.** Corner tracking on one target can't tell whether the camera or the target moved; both shift the target in the image.
**How we found it.** I first attributed it to camera roll, from the rotation alone. The user noted that real creep would look like a trend, not jumps. Phase correlation of static background tiles showed the camera within ±0.15 px while the board moved +0.57 px relative to the scene.
**The fix.** `stability_check.py` now tracks the background and reports board − background.
**Takeaway.** Always measure a reference that moves only with the camera before blaming the camera, and check the time series before naming a mechanism.

## 5. The camera-motion correction that had the wrong sign

*Logged 2026-10-30, Phase 2 step 5b(a).*

**Symptom.** In a 30-min static run the gripper tag drifted −1.25 px in y while the "static background" reportedly drifted +1.44 px, so "tag minus background" said the tag moved −2.7 px (≈ 0.3 mm) with the arm unpowered.
**What was going on.** The camera moved; tag and background moved together. The background estimate had its sign flipped.
**Why it happens.** `cv2.phaseCorrelate(a, b)` returns how far the content moved from `a` to `b`. The code assumed the opposite convention and negated it. With a still camera (bg ≈ 0) the bug is invisible, which is why the 10-28 runs looked right.
**How we found it.** The new PnP pipeline measures the iPad and the tag independently in the camera frame: both shifted ~+100 µm x / −130 µm y together. That contradicted the background tracker. A synthetic 2 px shift confirmed the convention.
**The fix.** Removed the negation; the line now documents the convention.
**Takeaway.** Check every sign convention against a synthetic known shift, and validate a correction against an independent measurement before trusting a difference of two large numbers.
