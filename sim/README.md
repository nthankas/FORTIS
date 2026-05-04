# sim/

Simulation work for FORTIS. Currently all sims are in `isaac/` (NVIDIA Isaac Sim 5.1, runs outside the dev container on the Windows host).

## Layout

```
sim/isaac/
  xdrive/                 Active drivetrain + arm sims
    canonical/            THE scripts to copy from
    deprecated/           Old scripts kept for history (do NOT copy from these)
    tools/                Analysis / measurement / standalone utilities
    lib/                  Shared modules (sim_config.py, arm_ik.py, arm_ik_v2.py)
    assets/               USD files (diiid_reactor.usd, omniwheels.usd)
    docs/                 Spec docs, summaries, R0 plan
    results/              Per-script output (results/<script_name>/...)
    CHANGELOG.md          Milestone history
  skid_steer_design/      Archived: proves skid-steer can't straddle the step
```

Rule: every script writes its output into `results/<script_name>/`. No flat dumps in `xdrive/`.

## Canonical scripts

- `xdrive/canonical/xdrive_realwheel.py` - chassis + real Kaya omni-wheel meshes (1 hub + 10 rollers per wheel as separate rigid bodies). 44-DOF articulation, CPU physics @ 360 Hz, TGS + CCD. Loads `assets/omniwheels.usd`. Supports `--reactor` for the DIII-D environment. Arched/octagonal skeleton `13.082" x 8.54" x 6.0"` with 3" chamfer faces, footprint `19.022" x 14.5"`, `2.0"` belly default.
- `xdrive/canonical/xdrive_reactor_arm_v2.py` - 4-DOF parallel-link arm on the realwheel chassis, flat-stowed. v2 build: heterogeneous motors (NEMA 17 + Cricket at J1/J3, NEMA 23 + EG23 at J2, Hitec D845WP at J4), 1.25"x1.38" rectangular CF tubes, camera at L4 midpoint. Only the 30" carbon-fiber configuration is the active build target; 24"/36" sweep modes remain for reference but are out of scope. v1 is deprecated, see `deprecated/scripts/xdrive_reactor_arm_v1.py`.

Any new sim that needs the FORTIS chassis copies its build path from `canonical/xdrive_realwheel.py`. Don't rebuild geometry from scratch and don't copy from anything in `deprecated/`.

## Tools

- `tools/orbit_torque.py` / `tools/orbit_torque_v2.py` - drive-torque profiling at orbit radii (v1 = legacy chassis dims, v2 = current arched skeleton)
- `tools/arm_continuous_sweep_v2.py` - raw torque sweep, collision-free teleport (v2 arm only; v1 deprecated)
- `tools/arm_sweep_filter_v2.py` - analytical post-filter on sweep CSVs (collision + tipping); v1 deprecated
- `tools/clearance_sweep.py`, `tools/measure_r0_port.py`, `tools/step_arch_optimizer.py`, `tools/test_drift.py` - standalone analytical / measurement utilities

v1 arm tooling (`arm_continuous_sweep.py`, `arm_sweep_filter.py`, `arm_stability_sweep.py`, `arm_ik.py`) lives in `deprecated/scripts/` with `_v1` suffix.

## Hardware spec parity

`canonical/xdrive_realwheel.py` and `lib/sim_config.py` match the current spec (~15" x 9" x 6", ODrive M8325s drivetrain). Mass constant in the sim is still `20.4 kg`; the current hardware target is ~40 lb (~18 kg). Refer to the OnShape model and BOM for ground-truth dimensions, not these scripts.

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

# Chassis + v2 arm (30" CF, the active build target)
IsaacSim/python.bat sim/isaac/xdrive/canonical/xdrive_reactor_arm_v2.py --gui

# Analytical clearance sweep (no Isaac Sim needed)
python sim/isaac/xdrive/tools/clearance_sweep.py
```

## Next sim work

Sim 2: R0 port entry. Plan in `xdrive/docs/R0_ENTRY_PLAN.md`. Not started.
