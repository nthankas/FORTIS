# sim/

Simulation work for FORTIS. Currently all sims are in `isaac/` (NVIDIA Isaac Sim 5.1, runs outside the dev container on the Windows host).

## Layout

```
sim/isaac/
  xdrive/                 Active drivetrain + arm sims
    canonical/            THE scripts to copy from (xdrive_realwheel.py, xdrive_reactor_arm_v3.py)
    deprecated/           Old scripts kept for history (do NOT copy from these)
      scripts/arm/        Deprecated arm scripts + IK (v1 arm, pose/torque/workspace sweeps)
      scripts/chassis/    Deprecated chassis scripts (early reactor sims, v1/v2 arm-on-chassis builds, rejected realwheel variants)
      results/            Old result dirs from deprecated scripts (data unreliable, kept for history)
    tools/                Analysis / measurement / standalone utilities
    lib/                  Shared modules (sim_config.py, arm_ik_v2.py, arm_ik_v3.py)
    assets/               USD files (diiid_reactor.usd, omniwheels.usd)
    docs/                 Spec docs, summaries, R0 plan
    results/              Per-script output (results/<script_name>/...)
    CHANGELOG.md          Milestone history
  skid_steer_design/      Archived: proves skid-steer can't straddle the step
```

Rule: every script writes its output into `results/<script_name>/`. No flat dumps in `xdrive/`.

## Canonical scripts

- `xdrive/canonical/xdrive_realwheel.py` -- chassis + real Kaya omni-wheel meshes (1 hub + 10 rollers per wheel as separate rigid bodies, after the 5-sphere roller-collider fix). 44-DOF articulation, CPU physics @ 360 Hz, TGS + CCD. Loads `assets/omniwheels.usd`. Supports `--reactor` for the DIII-D environment. Rectangular skeleton `13.082" x 8.54" x 6.0"`, footprint `19.022" x 14.5"` with wheels flush at corners (no chamfer), `2.0"` belly default.
- `xdrive/canonical/xdrive_reactor_arm_v3.py` -- **active build target.** 4-DOF parallel-link arm on the realwheel chassis, flat-stowed. Single-config: 30" carbon-fiber arm, always loaded with 3 lb payload. No `--24arm` / `--36arm` / `--metal` / `--armloaded` flags -- those collapsed into one fixed build. Heterogeneous motors (NEMA 17 + Cricket J1/J3, NEMA 23 + EG23 + adapter J2, Hitec D845WP J4), 1.128"x1.128" CF tubes, OAK-D Pro camera at L4 midpoint. v2 (`xdrive_reactor_arm_v2.py`) was deprecated when the heavier J2 stack landed -- see `deprecated/scripts/chassis/xdrive_reactor_arm_v2.py` for back-comparison.

Any new sim that needs the FORTIS chassis copies its build path from `canonical/xdrive_realwheel.py`. Don't rebuild geometry from scratch and don't copy from anything in `deprecated/`.

## Tools

- `tools/arm_continuous_sweep_v3.py` -- raw torque sweep, collision-free teleport (v3 arm). v2 (`arm_continuous_sweep_v2.py`) is kept alongside for back-comparison; v1 is deprecated.
- `tools/arm_sweep_filter_v3.py` -- analytical post-filter on v3 sweep CSVs (collision + tipping). v2 kept; v1 deprecated.
- `tools/arm_sweep_plot_v3.py` -- plots and tables for the v3 sweep (poloidal / stability / torque histograms / per-joint torque tables).
- `tools/sweep_orbit_realwheel.py` -- orbit-mode torque sweep on the canonical realwheel chassis. Used to validate the 5-sphere roller collider against the previous single-sphere baseline; results live under `results/orbit_realwheel_5sphere/` and `results/orbit_realwheel_singlesphere/`.
- `tools/clearance_sweep.py`, `tools/measure_r0_port.py`, `tools/test_drift.py` -- standalone analytical / measurement utilities (no Isaac Sim needed).

Older tooling (`arm_continuous_sweep_v1.py`, `arm_sweep_filter_v1.py`, `arm_stability_sweep_v1.py`, `arm_ik_v1.py`, `orbit_torque.py`, `orbit_torque_v2.py`, `step_arch_optimizer.py`, `sweep_orbit_variant.py`) lives under `deprecated/scripts/arm/` or `deprecated/scripts/chassis/` -- see `deprecated/README.md` for what each was and why it was retired.

## Hardware spec parity

`canonical/xdrive_realwheel.py` and `lib/sim_config.py` match the current spec (~15" x 9" x 6", ODrive M8325s drivetrain).

Mass budget (40 lb total robot, ~18.144 kg):

| Component | Mass | Source |
|---|---|---|
| Chassis body (arm lumped on top) | 14.144 kg | `canonical/xdrive_realwheel.py` `CHASSIS_MASS` |
| Wheels | 4 x 1.0 kg | `canonical/xdrive_realwheel.py` `WHEEL_MASS` |
| Total | 18.144 kg (= 40 lb) | |

Sweep range for parameter studies: `lib/sim_config.py` `MASS_VALUES_KG = [13.6, 18.1, 22.7]` (30 / 40 / 50 lb), default 18.1 kg. The arm-IK modules (`lib/arm_ik_v3.py`, deprecated `lib/arm_ik_v2.py`) carry their own `CHASSIS_MASS = 20.4` constant for analytical tipping calculations -- v3 is the active value, v2 is kept for back-comparison only.

Refer to the OnShape model and BOM for ground-truth dimensions, not these scripts.

## Reactor environment

- USD: `xdrive/assets/diiid_reactor.usd` (OnShape import, triangle mesh collision applied manually in the GUI - **do not regenerate**)
- Ground-truth mesh (not in git): `E:/Capstone/assets/halfReactor.stl`
- Key radii (in `xdrive/lib/sim_config.py`): inner floor 45.8-54.8", step at 55.5", outer floor 56.2-70.3", step height 4.5", central column r ~ 38"
- Z heights: outer floor -49.3", inner floor -53.8"

## Why x-drive (not skid-steer)

The robot straddles a 4.5" step between the inner and outer reactor floor. Skid-steer point turns while straddling this step were tested in 60 configurations - only 3 completed (5%), all with dangerous tilt and drift. See `skid_steer_design/ANALYSIS.md`.

## Running

Requires Isaac Sim 5.1 on the Windows host (not in the dev container).

```bash
# Canonical chassis on flat ground
IsaacSim/python.bat sim/isaac/xdrive/canonical/xdrive_realwheel.py --gui

# Canonical chassis inside reactor (straddling the step)
IsaacSim/python.bat sim/isaac/xdrive/canonical/xdrive_realwheel.py --gui --reactor

# Chassis + v3 arm (30" CF + 3 lb loaded -- the active build target)
IsaacSim/python.bat sim/isaac/xdrive/canonical/xdrive_reactor_arm_v3.py --gui
IsaacSim/python.bat sim/isaac/xdrive/canonical/xdrive_reactor_arm_v3.py --gui --reactor
IsaacSim/python.bat sim/isaac/xdrive/canonical/xdrive_reactor_arm_v3.py --gui --step

# v3 torque sweep + post-filter + plots
IsaacSim/python.bat sim/isaac/xdrive/tools/arm_continuous_sweep_v3.py
python sim/isaac/xdrive/tools/arm_sweep_filter_v3.py
python sim/isaac/xdrive/tools/arm_sweep_plot_v3.py

# Analytical clearance sweep (no Isaac Sim needed)
python sim/isaac/xdrive/tools/clearance_sweep.py
```

## Next sim work

Sim 2: R0 port entry. Plan in `xdrive/docs/R0_ENTRY_PLAN.md`. Not started.
