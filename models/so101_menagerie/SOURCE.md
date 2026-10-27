# Source of this model (vendored, do not edit in place)

- Upstream: https://github.com/google-deepmind/mujoco_menagerie, folder `robotstudio_so101/`
- Commit: `f3b6013629c7b94f8ee93fd977b161fa8350f9bc` (2026-10-23)
- Derived upstream from The Robot Studio `so101_new_calib.xml` (see README.md); joint zero = LeRobot's
  "middle of range" calibration convention. Apache-2.0 (LICENSE).
- Copied 2026-10-27 for Phase 1 step 7 (tick→model mapping) and step 8 (FK cross-check, collision model).
  Not copied: `so101.png`, `scene_box.xml`.
- To change: re-vendor from a new upstream commit and record it here; never hand-edit these files.
  Our own additions (table plane, etc.) go in separate files that `<include>` these.
