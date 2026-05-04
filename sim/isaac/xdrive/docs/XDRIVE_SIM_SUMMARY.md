# X-Drive Simulation

## Robot Model

- **Chassis**: 15" x 9" x 6" octagonal prism, 3" chamfers, 18.1 kg (40 lbs)
- **Wheels**: 4x omni wheels at 45-degree chamfered corners (X-drive config)
- **Simulation wheels**: 180mm dia sphere-approximated rollers (2 rows x 12 rollers x 2-3 spheres each)
- **Target hardware**: AndyMark 8" Dualie Plastic (am-0463), ODrive M8325s 100KV direct drive (no gearbox)

## Scripts

| Script | Purpose |
|--------|---------|
| `xdrive_o3dyn.py` | Flat ground driving with keyboard control and orbit mode |
| `xdrive_reactor.py` | Driving inside the DIII-D reactor geometry |
| `xdrive_r0_entry_v2.py` | R0 port entry with arched chassis + PID tether (WIP) |
| `test_drift.py` | Automated drift/accuracy testing across speeds and directions |
| `torque_sweep.py` | Headless torque measurement sweep (GPU, fresh stage per test) |
| `clearance_sweep.py` | Analytical underbelly clearance calculator (no simulation) |
| `sim_config.py` | Shared configuration constants |
| `measure_r0_port.py` | R0 port geometry extraction from reactor USD |

## Running

```
E:\FORTIS\IsaacSim\python.bat <script.py> --gui
E:\FORTIS\IsaacSim\python.bat <script.py> --headless
```

## Controls (GUI mode)

| Key | Action |
|-----|--------|
| Arrows | Forward/back/strafe |
| Q/E | Rotate CCW/CW |
| +/- | Speed up/down |
| O | Toggle orbit (OFF/CCW/CW) |
| 9/0 | Orbit radius -/+ |
| R | Reset position |
| P | Print full state |
| Space | Emergency stop |

## Key Results

- Forward 0.2 m/s: 3.1% lateral drift, +/-4 deg yaw oscillation
- Peak wheel torque: 6.1 Nm (overestimate -- real rollers spin freely)
- Estimated real-world peak: 3-4 Nm with 13 Nm motor capacity (3-4x margin)
- Stable at rest, no yaw drift

## Reactor Environment

`diiid_reactor.usd` -- triangle mesh collision applied via Physics API Editor in the Isaac Sim GUI. Do not regenerate this file programmatically.
