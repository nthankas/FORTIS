# Orbit torque sweep — 5-sphere roller collider, results summary

X-drive on the canonical realwheel chassis (1 hub + 5-sphere roller chain
per wheel) orbiting at R = 1.59 m around the reactor centre, straddling
the 4.5" step. Open-loop wheel velocities (one IK call after settle, no
closed-loop position tracking).

## Run config

| Parameter | Value |
|---|---|
| Chassis mass (with arm flat-stowed lump) | 16.659 kg = 10.144 chassis + 6.515 arm |
| Wheels | 4 × 1.0 kg |
| Total robot | 20.659 kg ≈ 45 lb |
| Wheel radius | 0.1016 m (4") |
| Orbit radius | 1.59 m (62.6", outer floor near step) |
| Settle (zero cmd before motion) | 5.0 s |
| Per-speed motion | 1 full orbit |
| Speeds | 0.10, 0.15, 0.20, 0.25, 0.30 m/s |
| Drive command | open-loop, constant `xdrive_ik(0, -speed, +speed/R)` |

The chassis is allowed to drift radially (R varies ~1.17–1.67 m across
all speeds). Tracking % is 18–24% by design — open-loop produces an
honest torque measurement instead of a closed-loop tracker fighting
position error.

## Per-wheel torque table

|  Speed (m/s)  |   Wheel   |   Mean (Nm)   |   P95 (Nm)   |   Peak (Nm)   |
|:--:|:--:|:--:|:--:|:--:|
| 0.10 | FL | 0.92 | 3.44 | 7.45 |
| 0.10 | FR | 0.91 | 2.70 | 6.01 |
| 0.10 | BL | 0.84 | 2.48 | 6.54 |
| 0.10 | BR | 0.61 | 2.06 | 6.70 |
| 0.15 | FL | 1.02 | 3.25 | 7.38 |
| 0.15 | FR | 0.91 | 2.63 | 7.63 |
| 0.15 | BL | 0.90 | 2.46 | 7.98 |
| 0.15 | BR | 0.66 | 2.28 | 7.34 |
| 0.20 | FL | 1.15 | 3.45 | 9.81 |
| 0.20 | FR | 0.99 | 2.93 | 8.68 |
| 0.20 | BL | 0.98 | 2.60 | 7.40 |
| 0.20 | BR | 0.71 | 2.40 | 7.46 |
| 0.25 | FL | 1.22 | 3.59 | 9.89 |
| 0.25 | FR | 0.99 | 3.02 | 10.64 |
| 0.25 | BL | 1.07 | 2.93 | 8.21 |
| 0.25 | BR | 0.72 | 2.65 | 8.29 |
| 0.30 | FL | 1.31 | 4.10 | 10.93 |
| 0.30 | FR | 1.23 | 3.49 | 9.62 |
| 0.30 | BL | 1.17 | 3.22 | 6.88 |
| 0.30 | BR | 0.92 | 3.04 | 10.35 |

## Headline numbers (worst wheel per speed)

| Speed (m/s) | Mean max (Nm) | P95 max (Nm) | Peak max (Nm) |
|:--:|:--:|:--:|:--:|
| 0.10 | 0.92 (FL) | 3.44 (FL) | 7.45 (FL) |
| 0.15 | 1.02 (FL) | 3.25 (FL) | 7.98 (BL) |
| 0.20 | 1.15 (FL) | 3.45 (FL) | 9.81 (FL) |
| 0.25 | 1.22 (FL) | 3.59 (FL) | 10.64 (FR) |
| 0.30 | 1.31 (FL) | 4.10 (FL) | 10.93 (FL) |

## Hardware comparison (ODrive M8325s, Kt ≈ 0.083 Nm/A)

| Threshold | Torque | Margin vs worst P95 (4.10 Nm @ 0.30) |
|:--:|:--:|:--:|
| 40 A continuous | 3.32 Nm | exceeded at 0.30 m/s |
| 60 A continuous | 4.98 Nm | comfortable across full sweep |
| 70 A continuous | 5.81 Nm | ample headroom |

P95 sits below the 60 A continuous rating across the entire 0.10–0.30 m/s
speed range. Peak excursions reach 10–11 Nm at 0.20 m/s and above —
short-duration roller-chatter spikes, well within transient capability of
the M8325s.

## Outputs in this directory

| File | Purpose |
|---|---|
| `orbit_realwheel_<speed>mps.csv` | Raw per-frame torques, position, yaw, R |
| `sweep_orbit_realwheel.json` | Aggregated mean / P95 / peak per wheel per speed |
| `torque_sweep_5sphere_vs_baseline.png` | Plot for slides (mean dashed + P95 solid, 40/60/70 A reference lines) |
| `plot_5sphere.py` | Plot generator |

The single-sphere baseline this used to be compared against has been
moved to `deprecated/results/orbit_realwheel_singlesphere/`. The 5-sphere
chain is the canonical roller model now.

## Pipeline

```
tools/sweep_orbit_realwheel.py   ->  per-speed CSVs + sweep_orbit_realwheel.json
plot_5sphere.py                  ->  torque_sweep_5sphere_vs_baseline.png
```

`sweep_orbit_realwheel.py` invokes `canonical/xdrive_realwheel.py` as a
subprocess at each speed with `--orbit-speed`, `--orbit-orbits`,
`--orbit-csv`, `--orbit-settle 5.0`, `--chassis-mass 16.659`. The orbit
branch in the canonical script computes the wheel velocities open-loop
and never updates them after settle — see
`canonical/xdrive_realwheel.py` orbit-mode block for the implementation.
