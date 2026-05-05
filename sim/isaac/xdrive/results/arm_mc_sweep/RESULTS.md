# Arm MC sweep v4 — results summary

Final arm sim. Headless Monte Carlo over the 4-DOF joint space on the v3
hardware build (30" CF arm, 3 lb payload, NEMA 17 + Cricket J1/J3,
NEMA 23 + EG23 J2, Hitec D845WP J4). Step environment (4.5" step, chassis
straddling).

## Run config

| Parameter | Value |
|---|---|
| Method | Uniform Monte Carlo over (J1, J2, J3, J4) |
| Samples | 5,000 |
| Seed | 42 |
| J1 / J2 / J3 range | ±180° each |
| J4 range | ±100° |
| Chassis mass | 3.629 kg (worst-case for tipping; total robot 14.144 kg) |
| Arm mass | 6.515 kg (articulated, 4 segments) |
| Wheels | 4 × 1.0 kg |
| Environment | step (4.5") |
| Sim runtime | 698 s headless |

The chassis mass is **deliberately set to the lightest plausible value in
the design range**. A lighter chassis tips more easily under arm
cantilever, so any pose that survives at 3.629 kg is conservatively safe
at the realistic 10.144 kg.

## Pose-space pass/fail

| Bucket | Count | % of 5,000 |
|---|---|---|
| Total samples | 5,000 | 100.0% |
| Geometric + reactor envelope valid | 2,014 | 40.3% |
| — chassis-collision rejects | 735 | 14.7% |
| — self-collision (L4↔L2) rejects | 319 | 6.4% |
| — floor-collision rejects | 1,326 | 26.5% |
| — reactor-envelope rejects | 1,720 | 34.4% |
| Valid + not tipped (physics) | 1,989 | 39.8% |
| Valid + tipped (worst-case mass) | 25 | 0.5% |

Of the 2,014 reachable / collision-free poses, **98.8% are stable** even
under worst-case chassis mass.

## Tipping margin (over the 2,014 valid poses)

| Stat | Value |
|---|---|
| Min margin | 0.022 m |
| Mean margin | 0.081 m |
| Max margin | 0.108 m |

Margin = horizontal distance from CG projection to the nearest support
polygon edge. Positive on every valid pose; the 25 physics-tipped poses
are still inside the support polygon analytically and tip due to dynamic
chassis rocking on the step (caught only by the physics check).

## Per-joint torque (over the 2,014 valid poses)

| Joint | Mean (Nm) | P95 (Nm) | Peak (Nm) | Hardware cont. rating | P95 / rating | Peak / rating |
|---|---|---|---|---|---|---|
| J1 | 0.03 | 0.05 | 4.52 | 12.0 Nm | 0.4% | 37.7% |
| J2 | 7.43 | 14.68 | 17.62 | 30.0 Nm | 48.9% | 58.7% |
| J3 | 3.26 | 5.97 | 6.74 | 12.0 Nm | 49.7% | 56.1% |
| J4 | 1.22 | 1.90 | 1.95 | 4.9 Nm | 38.8% | 39.7% |

Hardware references:
- J1, J3: NEMA 17 + Cricket MK II 25:1, ~12 Nm continuous
- J2: NEMA 23 + EG23 20:1 + adapter, 30 Nm continuous
- J4: Hitec D845WP servo, 4.9 Nm

Every joint sits below 60% of its continuous rating at peak, with P95 at
or below ~50% — meaningful headroom across the reachable joint space.

## Outputs in this directory

| File | Purpose |
|---|---|
| `30in_loaded_step_v4_mc5000.csv` | Raw MC sweep, 5,000 rows, all joint torques + chassis state |
| `30in_loaded_step_v4_mc5000_filtered.csv` | Same poses with filter_status and tipping flags appended |
| `30in_loaded_step_v4_mc5000_summary.json` | Sweep config + headline counts |
| `30in_loaded_step_v4_mc5000_filter_summary.json` | Filter pass/reject + tipping margin stats |
| `poloidal_v4.png` | Reachable EE points overlaid on DIII-D poloidal cross-section |
| `sampling_hist_v4.png` | Joint sampling histogram (verification of uniform draws) |
| `torque_table_v4.png` | Rendered torque table for slides |
| `torques_v4.png` | Per-joint torque bar chart |

## Pipeline

```
arm_mc_sweep_v4.py    -> raw CSV (Isaac Sim, headless)
arm_mc_filter_v4.py   -> *_filtered.csv + filter_summary.json
arm_mc_plot_v4.py     -> poloidal/hist/table/torques PNGs
```

The filter and plot scripts are run with `--chassis-mass-kg 10.144` (the
realistic mass) so the tipping margins and analytical comparisons are
reported against the real chassis, while the sim physics ran at the
worst-case 3.629 kg. This asymmetry is intentional — see
`tools/arm_mc_sweep_v4.py` docstring.
