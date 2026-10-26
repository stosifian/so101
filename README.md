# SO-101

Bring-up, metrology characterization, and follow-on experiments for a Seeed SO-ARM101 Pro arm.

- Spec: [`so101-metrology-plan.md`](so101-metrology-plan.md)
- Roadmap: [`docs/ROADMAP.md`](docs/ROADMAP.md)
- Hardware log: [`docs/hardware_log.md`](docs/hardware_log.md)

## Setup
```bash
uv sync                 # creates .venv (Python 3.12, arm64) from uv.lock
uv run pytest           # hardware-free tests
```

## Scripts
- `scripts/readonly/` never enables torque. Safe to run with the arm powered.
- `scripts/motion/` can move the arm. Needs `--arm-moves`. Support the arm and keep the 12 V switch within reach.

## Pinned versions
Exact versions are in `uv.lock`, and every run header records them. Baseline at bootstrap:

| Component | Version |
|-----------|---------|
| Python | 3.12.15 (arm64) |
| uv | 0.13.5 |
