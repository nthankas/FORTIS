# FORTIS — Tokamak Reactor Inspection Robot

Remotely operated inspection robot for the DIII-D tokamak fusion reactor at General Atomics. Enters through a 15.75" access tunnel, descends via tether through the R0 port, and drives on the reactor floor using an X-drive (4 omni wheels at 45-degree corners). Carries a 4-DOF robotic arm for camera inspection and thermocouple heating.

## Project Structure

```
simulation/isaac/
    xdrive/                         # Active: X-drive omni wheel simulation
    skid_steer_rejection/           # Archived: proves skid-steer can't work
control/                            # Robot control software (future)
firmware/                           # ESP32 firmware (future)
inverse_kinetmatic_solver.py        # CoppeliaSim IK solver (reference)
```

## Drive System

- **Configuration**: X-drive (4 omni wheels at 45-degree chamfered corners)
- **Chassis**: 15" x 9" x 6" octagonal prism, 3" chamfers, 40 lbs nominal
- **Wheels**: AndyMark 8" Dualie Plastic Omni (am-0463)
- **Motors**: REV NEO Brushless + 5:1 MAXPlanetary (13 Nm at wheel)
- **Friction**: Rubber on graphite, mu = 0.3 to 0.5

## Why X-Drive (not Skid-Steer)

The robot straddles a 4.5" step between the inner and outer reactor floor. Skid-steer
point turns while straddling this step were tested in simulation -- only 3 of 60
configurations completed the turn (5% pass rate), all with dangerous tilt and drift.
See `simulation/isaac/skid_steer_rejection/ANALYSIS.md` for the full writeup.

## Reactor Environment

- **Access tunnel**: 15.75" x 15.75"
- **R0 port**: 22" x 35.5" opening
- **Floor**: Two surfaces separated by a 4.5" step; robot straddles the step
- **Slope**: Interior wall ~35 degrees -- robot cannot drive on it, tether carries weight

## Running Simulations

Requires NVIDIA Isaac Sim 5.1.

```bash
# Flat ground driving
IsaacSim\python.bat simulation/isaac/xdrive/xdrive_o3dyn.py --gui

# Inside reactor
IsaacSim\python.bat simulation/isaac/xdrive/xdrive_reactor.py --gui

# R0 port entry maneuver
IsaacSim\python.bat simulation/isaac/xdrive/xdrive_r0_entry_v2.py --gui

# Analytical clearance sweep (no Isaac Sim needed)
python simulation/isaac/xdrive/clearance_sweep.py
```
