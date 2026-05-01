# sim/

Simulation work for FORTIS. Currently all sims are in `isaac/` (NVIDIA Isaac Sim 5.1, runs outside the dev container on the Windows host).

## Layout

```
sim/isaac/
  xdrive/                 Active drivetrain + arm sims
    canonical/            THE scripts to copy from
    deprecated/           Old scripts kept for history (do NOT copy from these)
    tools/                Analysis / measurement / standalone utilities
    lib/                  Shared modules (sim_config.py, arm_ik.py)
    assets/               USD files (diiid_reactor.usd, omniwheels.usd)
    docs/                 Spec docs, summaries, R0 plan
    results/              Per-script output (results/<script_name>/...)
    CHANGELOG.md          Milestone history
  skid_steer_design/      Archived: proves skid-steer can't straddle the step
```

Rule: every script writes its output into `results/<script_name>/`. No flat dumps in `xdrive/`.

## Canonical scripts

- `xdrive/canonical/xdrive_realwheel.py` - chassis + real Kaya omni-wheel meshes (1 hub + 10 rollers per wheel as separate rigid bodies). 44-DOF articulation, CPU physics @ 360 Hz, TGS + CCD. Loads `assets/omniwheels.usd`. Supports `--reactor` for the DIII-D environment.
- `xdrive/canonical/xdrive_reactor_arm.py` - 4-DOF parallel-link arm on the realwheel chassis, flat-stowed. Keyboard control + torque readout.

Any new sim that needs the FORTIS chassis copies its build path from `canonical/xdrive_realwheel.py`. Don't rebuild geometry from scratch and don't copy from anything in `deprecated/`.

## Known stale spec values

`STATE.md` (deleted) and the in-script constants in `canonical/xdrive_realwheel.py` carry chassis dims `15.354" x 9.353" x 7.1"` and mass `20.4 kg`, which predate the current ~15" x 9" x 6", ~40 lb hardware spec and the move from REV NEO motors to ODrive M8325s. The sim physics still runs; it's the absolute numbers that are stale. Refer to the OnShape model and BOM for ground-truth dimensions, not these scripts.

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

# Chassis + flat-stowed arm in reactor
IsaacSim/python.bat sim/isaac/xdrive/canonical/xdrive_reactor_arm.py --gui

# Analytical clearance sweep (no Isaac Sim needed)
python sim/isaac/xdrive/tools/clearance_sweep.py
```

## Next sim work

Sim 2: R0 port entry. Plan in `xdrive/docs/R0_ENTRY_PLAN.md`. Not started.
