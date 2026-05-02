# xdrive/ Changelog

High-level history of the X-drive simulation. Details live in git log; this
file tracks the milestones that changed the canonical model.

---

## 2026-05-01 — V2 arm + step stability + rectangular chassis skeleton (cb74dcc)
- Chassis skeleton pivot in `canonical/xdrive_realwheel.py` and
  `lib/sim_config.py`: `13.082" x 8.54" x 6.0"` rectangular skeleton (was
  `15.354" x 9.353" x 7.1"` octagonal-prism w/ chamfer). Total footprint
  with wheels: `19.022" x 14.5"`, wheels flush at corners, no chamfer.
  `BELLY_HEIGHT` default now `2.0"` (was `2.5"`).
- V2 arm system added under `canonical/xdrive_reactor_arm_v2.py` +
  `lib/arm_ik_v2.py`: heterogeneous motors, `1.25" x 1.38"` rectangular CF
  tubes, camera moved to L4. v1 arm kept canonical for back-comparison.
- V2 sweep tools: `tools/arm_continuous_sweep_v2.py`,
  `tools/arm_sweep_filter_v2.py`. Initial 30in-loaded-step results +
  `sweep_report_v2.md` under `results/arm_continuous_sweep/`.
- Step stability sweep: `tools/arm_stability_sweep.py` reads filtered v1
  CSV and runs physics tipping. New `tipping_at_j1_tilted()` in
  `lib/arm_ik.py` does tilt-corrected CG projection for the step-straddle
  case. Results in `results/arm_stability_sweep/`.
- `canonical/xdrive_reactor_arm.py` (v1): `--metal`/`--cf` flag to swap
  link material (CF default).
- `tools/orbit_torque_v2.py` added: orbit torque profiling on the new
  rectangular skeleton.
- Branches consolidated: `arm-sweep-v2`, `arm-v2-torque-report`, `dev`,
  `feature/comms-library`, `orbit-torque-profiling`, `phase2-results`,
  `feature/docker-devcontainer` work all in `main` now. Old branches
  preserved as `archive/<name>` tags on origin.

## 2026-04-04 — Folder reorg + flat-stowed arm
- Restructured `xdrive/` into `canonical/`, `deprecated/`, `tools/`, `lib/`,
  `assets/`, `docs/`, `results/`. Every script now writes to
  `results/<script_name>/`.
- Renamed `skid_steer_rejection/` → `skid_steer_design/`.
- `xdrive_reactor_arm.py` (Sim3 phase 1): rewrote stowed pose from vertical
  zigzag to **flat on chassis top**. Mount moved to the chassis back edge;
  L2 cantilevers forward, L3 folds back, L4 folds forward. CG is now near
  chassis center at spawn, so chassis tilt on the step no longer tips the arm
  into the reactor.

## 2026-04-01 — Orbit torque + R0 contact reporting (6755f38)
- Added `tools/orbit_torque.py` for torque profiling around reactor orbit radii.
- R0 entry v2 logs belly-roller contact forces during lip transit.

## 2026-03-28 — Real Kaya omni-wheel model becomes canonical (a22615d)
- `canonical/xdrive_realwheel.py` replaces the old sphere-roller approximation.
  44-DOF articulation (1 chassis + 4 hubs + 40 rollers) on CPU physics @ 360 Hz
  with TGS + CCD. Loads real wheel meshes from `assets/omniwheels.usd`.
- Belly arch + 3" flat center + motor mount flats, `convexDecomposition` collider.
- Earlier `xdrive_o3dyn.py`, `xdrive_reactor.py`, `xdrive_reactor_v2.py` moved
  to `deprecated/` — kept for history, do not copy from.

## 2026-03-18 — Reactor v2 + belly rollers (1498b32)
- Added 7 passive belly rollers in arch zone for R0 lip transit (graphite
  protection: all underbelly contact must roll, never slide).
- `xdrive_reactor_v2.py` as intermediate chassis/arch tuning step.

## 2026-03-16 — Phase 3 X-drive canonical established (b7d016c, b0e6434)
- First X-drive sim with correct chassis dimensions and reactor USD import.
- Replaced skid-steer direction entirely — skid-steer moved to
  `skid_steer_design/` with full rejection analysis.
