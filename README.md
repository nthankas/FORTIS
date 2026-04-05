# FORTIS — Tokamak Reactor Inspection Robot

Remotely operated inspection robot for the DIII-D tokamak fusion reactor at General Atomics. Enters through a 15.75" access tunnel, descends via tether through the R0 port, and drives on the reactor floor using an X-drive (4 omni wheels at 45-degree corners). Carries a 4-DOF robotic arm for camera inspection and thermocouple heating.

**Read `STATE.md` before touching any simulation script.** It is the canonical registry of which chassis/wheel/arm model is active and which scripts are deprecated.

## Project Structure

```
simulation/isaac/
    xdrive/
        canonical/     # Canonical chassis + arm scripts
        deprecated/    # Old models kept for history — do NOT copy
        tools/         # Analysis / measurement utilities
        lib/           # Shared modules (sim_config.py)
        assets/        # USD files (reactor, omniwheels)
        docs/          # Spec docs and plans
        results/       # Per-script outputs (results/<script>/)
    skid_steer_design/ # Archived: proves skid-steer can't work
control/               # Robot control software (future)
firmware/              # ESP32 firmware (future)
inverse_kinetmatic_solver.py  # CoppeliaSim IK solver (reference)
```

## Drive System

- **Configuration**: X-drive (4 omni wheels at 45-degree chamfered corners)
- **Chassis**: 15.354" x 9.353" x 7.1" octagonal prism, 3" chamfers, 20.4 kg
- **Wheels**: AndyMark 8" Dualie Plastic Omni (am-0463)
- **Motors**: REV NEO 2.0 brushless + REV MAXPlanetary 9:1 (30.38 Nm at wheel @ 90% eff)
- **Friction**: Rubber on graphite, mu = 0.3 to 0.5

## Why X-Drive (not Skid-Steer)

The robot straddles a 4.5" step between the inner and outer reactor floor. Skid-steer
point turns while straddling this step were tested in simulation -- only 3 of 60
configurations completed the turn (5% pass rate), all with dangerous tilt and drift.
See `simulation/isaac/skid_steer_design/ANALYSIS.md` for the full writeup.

## Reactor Environment

- **Access tunnel**: 15.75" x 15.75"
- **R0 port**: 22" x 35.5" opening
- **Floor**: Two surfaces separated by a 4.5" step; robot straddles the step
- **Slope**: Interior wall ~35 degrees -- robot cannot drive on it, tether carries weight

## Running Simulations

Requires NVIDIA Isaac Sim 5.1.

```bash
# Canonical chassis on flat ground
IsaacSim\python.bat simulation/isaac/xdrive/canonical/xdrive_realwheel.py --gui

# Canonical chassis inside reactor (straddling the step)
IsaacSim\python.bat simulation/isaac/xdrive/canonical/xdrive_realwheel.py --gui --reactor

# Sim3 phase 1 — chassis + flat-stowed arm in reactor
IsaacSim\python.bat simulation/isaac/xdrive/canonical/xdrive_reactor_arm.py --gui

# Analytical clearance sweep (no Isaac Sim needed)
python simulation/isaac/xdrive/tools/clearance_sweep.py
```
