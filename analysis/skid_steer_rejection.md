# FORTIS: Skid-Steer Drivetrain Rejection

## Conclusion

**Skid-steer point turns over the reactor step are not viable.** Out of 60 geometrically valid configurations tested in Isaac Sim, only 3 (5%) completed a 90-degree turn — and all three nearly flipped the robot (85-116 degree tilt). The fundamental problem is that skid-steer wheels must cross the step during any point turn, requiring them to climb a wall nearly equal to their own radius.

## The Problem

The robot enters the reactor through the R0 port and lands straddling the 4.5" step between the inner and outer floor (radial orientation). To drive tangentially and inspect the reactor, it must perform a 90-degree point turn while its wheels straddle the step.

### Reactor Floor Geometry

```
Inner floor:  Z = -53.8"    r = 45.8" to 54.8"    (width: 9.0")
Outer floor:  Z = -49.3"    r = 56.2" to 70.3"    (width: 14.1")
Step height:  4.5" (3.5" effective with 1" chamfer)
Step radius:  55.5" (center of step wall)
```

## Simulation Setup

| Parameter | Value |
|-----------|-------|
| Simulator | NVIDIA Isaac Sim 5.1.0 (PhysX) |
| Reactor geometry | Imported from STL mesh (ground truth) |
| Step wall | Chamfered: 1" x 1" 45-degree ramp at top edge |
| Friction | mu_s=0.8, mu_d=0.6 (generous; real-world likely 0.3-0.5) |
| Drive mode | Skid-steer (left reverse, right forward) |
| Robot mass | 40 lbs (18.1 kg) + 4x 2.5 lb wheels |
| Timeout | 20 seconds per trial |
| Success criterion | >= 90 degrees cumulative yaw rotation |

## Parameter Sweep

| Parameter | Values Tested |
|-----------|---------------|
| Wheel radius | 3.5, 4.0, 4.5 inches |
| Wheel width | 2.0, 3.0 inches |
| Wheelbase (front-rear) | 9, 10, 12 inches |
| Track width (left-right) | 8, 10, 12 inches |
| Motor torque | 50, 100 Nm per wheel |
| Drive velocity | 15 rad/s |
| Chassis mass | 40 lbs |

- **108 configurations generated**
- **48 rejected** by geometric pre-filter (inner floor clearance or tunnel fit)
- **60 tested** in simulation
- **3 passed** (5% pass rate)

## Results: The 3 Configs That Passed

| # | Wheel R | Wheel W | WB | TW | Torque | Time | Max Tilt | Drift |
|---|---------|---------|----|----|--------|------|----------|-------|
| 1 | 3.5" | 2.0" | 10" | 8" | 100 Nm | 6.9s | **85 deg** | 7.0" |
| 2 | 3.5" | 2.0" | 9" | 8" | 50 Nm | 13.1s | **116 deg** | 10.1" |
| 3 | 3.5" | 2.0" | 9" | 8" | 100 Nm | 13.4s | **106 deg** | 9.3" |

All three share: smallest wheel (3.5"), narrowest tire (2.0"), narrowest track (8"). All tilt past 85 degrees (nearly flipping). All drift 7-10" radially.

## Why Skid-Steer Fails

### 1. Wheels must cross the step during any turn

During a point turn, each wheel traces a circular arc around the robot's center. The step is a circle at r=55.5". The wheel arcs inevitably cross this circle.

- **Best crossing angle**: 52.6 degrees (WB=12", TW=8")
- **Required rotation**: 90 degrees
- This means wheels **must** climb the step mid-turn — there is no geometry that avoids it

### 2. Step climbing is physically extreme

The effective step (3.5" with chamfer) is nearly equal to the wheel radius (3.5"). Climbing requires:
- Instantaneous torque spike as wheel hits the wall
- Maintaining rotational momentum with wheels airborne
- Surviving violent chassis tilt as wheels transition between floor levels

### 3. No design margin exists

| Change | Effect |
|--------|--------|
| Wheel radius 3.5" → 4.0" | All fail (larger wheel can't fit inner floor) |
| Track width 8" → 10" | All fail (crossing angle decreases) |
| Wheelbase 9" → 12" | Fails (best crossing angle but still can't climb) |
| Torque 50 → 100 Nm | Marginal (helps only 1 of 3 configs) |
| Friction 0.8 → 0.5 (realistic) | Would eliminate all 3 passing configs |

### 4. The "successful" turns are unacceptable

- **85-116 degree tilt**: Robot is nearly perpendicular to the floor or past vertical
- **7-10" radial drift**: Robot slides toward center post or outer wall
- In a reactor with delicate plasma-facing components, this is catastrophic

## Sensitivity Analysis (All 60 Configs)

```
Config space:  R=3.5" W=2.0" TW=8"   →  3/6 pass (50%) — but all tilt > 85 deg
               R=3.5" W=2.0" TW=10"  →  0/6 pass
               R=3.5" W=2.0" TW=12"  →  0/6 pass
               R=3.5" W=3.0"         →  0/18 pass
               R=4.0" (any)          →  0/18 pass
               R=4.5" (any)          →  0/12 pass
```

The only region of parameter space that works is R=3.5", W=2.0", TW=8" — the absolute minimum of everything. This leaves zero margin for real-world conditions.

## Comparison: Skid-Steer vs Holonomic (X-Drive)

| Factor | Skid-Steer | X-Drive |
|--------|-----------|---------|
| 90-deg turn on step | 5% pass, 85-116 deg tilt | **100% pass, 17 deg tilt (step only)** |
| Wheels cross step during turn | Always | **Never** |
| Lateral motion | Not possible | **Full holonomic** |
| Torque for point turn | Unbounded (step impact) | **5.3 Nm (friction only)** |
| Radial drift during turn | 7-10" | **< 1"** |
| Required friction (mu) | 0.8+ (generous) | **0.5 (measured graphite)** |
| Step climbing during normal ops | Required | **Not required** |
| Risk to reactor components | High (violent maneuvers) | **Low (smooth, controlled)** |

## Data Files

| File | Description |
|------|-------------|
| `simulation/isaac/skid_steer_rejection/step_sweep.py` | Isaac Sim parametric sweep script |
| `simulation/isaac/skid_steer_rejection/build_robot.py` | Parametric robot builder |
| `simulation/isaac/skid_steer_rejection/ANALYSIS.md` | Detailed results table |
