# FORTIS Torque Analysis: 5:1 vs 9:1 Gear Ratio

## Motor: REV NEO 2.0 Brushless

| Spec | Value |
|------|-------|
| Free speed | 5,676 RPM |
| Stall torque | 3.75 Nm |
| Stall current | 166 A @ 12V |
| Dimensions | 60 mm dia x 48 mm |
| Mass | 0.94 lbs (425 g) |

## Gearbox: REV MAXPlanetary

| Parameter | 5:1 (Single Stage) | 9:1 (Two Stage: 3x3) |
|-----------|--------------------|-----------------------|
| Gear ratio | 5:1 | 9:1 |
| Wheel torque (stall) | 18.75 Nm | 33.75 Nm |
| Wheel speed (free) | 1,135 RPM / 118.9 rad/s | 631 RPM / 66.0 rad/s |
| Max ground speed (8" wheel) | 12.08 m/s (27.0 mph) | 6.71 m/s (15.0 mph) |
| Stage count | 1 | 2 |
| Backdrive torque | Lower | Higher |

## Measured Torque Demand (Isaac Sim)

| Maneuver | Peak Torque (sim) | Estimated Real | Notes |
|----------|-------------------|----------------|-------|
| Forward 0.2 m/s flat | 6.1 Nm | ~5.1 Nm | Sim overestimates due to sphere contact model |
| Forward 0.1 m/s flat | 2.5 Nm | ~2.0 Nm | Low speed, minimal dynamics |
| Orbit 0.2 m/s on step | 4.0-10.7 Nm | ~3.5-9.0 Nm | Asymmetric load from 17 deg tilt |
| Step straddle (static) | 0.6 Nm | ~0.5 Nm | Holding position on tilted surface |

Simulation conditions: 240 Hz PhysX, TGS solver, mu=0.5 graphite friction, 45 lbs robot mass.

## Torque Margin Analysis

### Gravity Loading on Step

The 4.5" step creates a 17-degree chassis tilt when straddling. Gravity component along the tilt:

```
Robot weight:  45 lbs = 200.1 N
Slope force:   200.1 x sin(17 deg) = 58.5 N
Per wheel:     58.5 / 4 = 14.6 N
Torque/wheel:  14.6 x 0.1016 m = 1.49 Nm  (gravity component only)
```

### Margin Table

| Condition | Demand | 5:1 Available | 5:1 Margin | 9:1 Available | 9:1 Margin |
|-----------|--------|---------------|------------|---------------|------------|
| Forward 0.2 m/s flat | 5.1 Nm | 18.75 Nm | **3.7x** | 33.75 Nm | **6.6x** |
| Orbit on step (peak) | 9.0 Nm | 18.75 Nm | **2.1x** | 33.75 Nm | **3.8x** |
| Gravity hold on step | 1.5 Nm | 18.75 Nm | **12.5x** | 33.75 Nm | **22.5x** |
| Worst case (orbit + slope) | ~10.5 Nm | 18.75 Nm | **1.8x** | 33.75 Nm | **3.2x** |

### Speed Requirement Check

| Parameter | 5:1 | 9:1 | Requirement |
|-----------|-----|-----|-------------|
| Max ground speed | 12.08 m/s | 6.71 m/s | 0.5 m/s max operational |
| Wheel RPM at 0.5 m/s | 47 RPM | 47 RPM | — |
| % of max RPM used | 4.1% | 7.5% | — |
| Speed margin | 24x | 13x | Both massively over-spec |

## Recommendation

**9:1 is the better choice for FORTIS.**

- The robot operates at extremely low speeds (0.1-0.5 m/s) where neither ratio is speed-limited
- The 9:1 doubles the torque margin from 1.8x to 3.2x in the worst-case orbit+slope scenario
- Higher backdrive resistance from the 9:1 provides passive braking when the tether lowers the robot through the R0 port
- The additional MAXPlanetary stage adds ~20mm length and ~50g mass per motor — negligible impact on packaging
- 5:1 margin of 1.8x on the step is uncomfortably thin given real-world unknowns (graphite dust, uneven surfaces, cable drag)

## Current vs Proposed Configuration

| | Current (5:1) | Proposed (9:1) |
|---|---|---|
| Wheel torque | 18.75 Nm | 33.75 Nm |
| Worst-case margin | 1.8x | 3.2x |
| Max speed | 12.08 m/s | 6.71 m/s |
| Speed utilization | 4.1% | 7.5% |
| Gearbox length | ~50 mm | ~70 mm |
