# FORTIS Phase 2: Skid-Steer Straddle Turn Analysis

## Conclusion: Skid-steer point turns over the reactor step are not viable. Switch to mecanum wheels.

---

## Test Description

The robot enters the reactor through the R0 port and lands on the outer floor straddling
the step in the **radial** direction (front-to-back across the step). To drive along the
reactor floor (tangentially), it must perform a **90-degree point turn** while its wheels
straddle the 4.5" step between the inner and outer floor levels.

This test sweeps robot geometry parameters to find configurations that can complete a
90-degree skid-steer point turn while straddling the step.

### Simulation Environment
- **Simulator**: NVIDIA Isaac Sim 5.1.0 (PhysX rigid body dynamics)
- **Reactor geometry**: Imported from STL mesh (ground truth), scale 0.024 m/inch
- **Step wall**: Chamfered (1" x 1" 45-degree ramp at top edge, reducing effective step from 4.5" to 3.5")
- **Friction**: Rubber-on-graphite mu_s=0.8, mu_d=0.6 (generous; real-world likely 0.3-0.5)
- **Drive mode**: Skid-steer (left wheels reverse, right wheels forward)
- **Timeout**: 20 seconds per trial

### Reactor Floor Geometry
```
Inner floor:  Z = -53.8"    r = 45.8" to 54.8"    (width: 9.0")
Outer floor:  Z = -49.3"    r = 56.2" to 70.3"    (width: 14.1")
Step height:  4.5" (effective 3.5" with 1" chamfer)
Step gap:     1.4" horizontal (r = 54.8" to 56.2")
Step radius:  55.5" (center of step wall)
```

---

## Parameters Swept

| Parameter       | Values Tested         | Unit   |
|----------------|-----------------------|--------|
| Wheel radius   | 3.5, 4.0, 4.5        | inches |
| Wheel width    | 2.0, 3.0             | inches |
| Wheelbase      | 9, 10, 12            | inches |
| Track width    | 8, 10, 12            | inches |
| Motor torque   | 50, 100              | Nm     |
| Drive velocity | 15                    | rad/s  |
| Chassis mass   | 40                    | lbs    |

**Total configs generated**: 108
**Configs after geometric filtering**: 60 (48 rejected for inner floor clearance or tunnel fit)
**Configs that completed 90 degrees**: 3 out of 60 (**5% pass rate**)

---

## Results: Configs That Passed

| # | Wheel R | Wheel W | Wheelbase | Track W | Torque | Time  | Peak Rate | Max Tilt | Drift  |
|---|---------|---------|-----------|---------|--------|-------|-----------|----------|--------|
| 1 | 3.5"    | 2.0"    | 10"       | 8"      | 100 Nm | 6.9s  | 306 deg/s | 85 deg   | 7.0"   |
| 2 | 3.5"    | 2.0"    | 9"        | 8"      | 50 Nm  | 13.1s | 153 deg/s | 116 deg  | 10.1"  |
| 3 | 3.5"    | 2.0"    | 9"        | 8"      | 100 Nm | 13.4s | 153 deg/s | 106 deg  | 9.3"   |

---

## Results: All Configs (Sorted by Crossing Angle)

```
  R     W    WB    TW   Torque  Cross   Pass?  Yaw     Max Tilt  Drift
 3.5   2.0   12     8    50Nm   52.6    NO      24       22       2.2"
 3.5   2.0   12     8   100Nm   52.6    NO      24       22       2.2"
 3.5   2.0   10     8    50Nm   48.0    NO      87       93       8.4"
 3.5   2.0   10     8   100Nm   48.0    YES     96       85       7.0"   <-- BEST
 4.0   2.0   10     8    50Nm   48.0    NO      45       27       3.3"
 4.0   2.0   10     8   100Nm   48.0    NO      45       27       3.3"
 4.5   2.0   10     8    50Nm   48.0    NO      36       27       3.5"
 4.5   2.0   10     8   100Nm   48.0    NO      36       27       3.5"
 3.5   2.0   12    10    50Nm   46.2    NO      51       22       2.2"
 3.5   2.0   12    10   100Nm   46.2    NO      51       22       2.2"
 3.5   3.0   12    10    50Nm   46.2    NO      42       22       2.2"
 3.5   3.0   12    10   100Nm   46.2    NO      42       22       2.2"
 3.5   2.0    9     8    50Nm   45.3    YES     91      116      10.1"
 3.5   2.0    9     8   100Nm   45.3    YES     92      106       9.3"
 4.0   2.0    9     8    50Nm   45.3    NO      39       31       4.2"
 4.0   2.0    9     8   100Nm   45.3    NO      39       31       4.2"
 3.5   2.0   10    10    50Nm   41.3    NO      11       28       3.5"
 3.5   2.0   10    10   100Nm   41.3    NO      11       28       3.5"
 3.5   3.0   10    10    50Nm   41.3    NO      10       27       3.5"
 3.5   3.0   10    10   100Nm   41.3    NO      10       27       3.5"
 4.0   2.0   10    10    50Nm   41.3    NO      13       27       3.5"
 4.0   2.0   10    10   100Nm   41.3    NO      13       27       3.5"
 4.0   3.0   10    10    50Nm   41.3    NO      13       27       3.6"
 4.0   3.0   10    10   100Nm   41.3    NO      13       27       3.6"
 4.5   2.0   10    10    50Nm   41.3    NO      34       27       3.6"
 4.5   2.0   10    10   100Nm   41.3    NO      34       27       3.6"
 4.5   3.0   10    10    50Nm   41.3    NO      27       27       3.6"
 4.5   3.0   10    10   100Nm   41.3    NO      27       27       3.6"
 3.5   2.0   12    12    50Nm   40.6    NO      14       22       2.3"
 3.5   2.0   12    12   100Nm   40.6    NO      14       22       2.3"
 3.5   3.0   12    12    50Nm   40.6    NO      25       22       2.4"
 3.5   3.0   12    12   100Nm   40.6    NO      25       22       2.4"
 3.5   2.0    9    10    50Nm   38.5    NO      27       31       4.2"
 3.5   2.0    9    10   100Nm   38.5    NO      27       31       4.2"
 3.5   3.0    9    10    50Nm   38.5    NO      20       31       4.3"
 3.5   3.0    9    10   100Nm   38.5    NO      20       31       4.3"
 4.0   2.0    9    10    50Nm   38.5    NO      25       31       4.2"
 4.0   2.0    9    10   100Nm   38.5    NO      25       31       4.2"
 4.0   3.0    9    10    50Nm   38.5    NO      36       31       4.3"
 4.0   3.0    9    10   100Nm   38.5    NO      36       31       4.3"
 3.5   2.0   10    12    50Nm   35.8    NO      10       27       3.6"
 3.5   2.0   10    12   100Nm   35.8    NO      10       27       3.6"
 3.5   3.0   10    12    50Nm   35.8    NO       9       27       3.7"
 3.5   3.0   10    12   100Nm   35.8    NO       9       27       3.7"
 4.0   2.0   10    12    50Nm   35.8    NO       7       27       3.6"
 4.0   2.0   10    12   100Nm   35.8    NO       7       27       3.6"
 4.0   3.0   10    12    50Nm   35.8    NO      19       27       3.7"
 4.0   3.0   10    12   100Nm   35.8    NO      19       27       3.7"
 4.5   2.0   10    12    50Nm   35.8    NO      54       27       3.7"
 4.5   2.0   10    12   100Nm   35.8    NO      54       27       3.7"
 4.5   3.0   10    12    50Nm   35.8    NO      36       27       3.7"
 4.5   3.0   10    12   100Nm   35.8    NO      36       27       3.7"
 3.5   2.0    9    12    50Nm   33.0    NO      19       31       4.3"
 3.5   2.0    9    12   100Nm   33.0    NO      19       31       4.3"
 3.5   3.0    9    12    50Nm   33.0    NO      14       31       4.4"
 3.5   3.0    9    12   100Nm   33.0    NO      14       31       4.4"
 4.0   2.0    9    12    50Nm   33.0    NO      32       31       4.4"
 4.0   2.0    9    12   100Nm   33.0    NO      27       31       4.3"
 4.0   3.0    9    12    50Nm   33.0    NO      47       32       4.5"
 4.0   3.0    9    12   100Nm   33.0    NO      47       32       4.5"
```

---

## Why Skid-Steer Fails

### 1. Fundamental geometry problem: wheels must cross the step
During a point turn, each wheel traces a circular arc. When straddling the step at r=55.5",
the turning circle forces wheels to cross from one floor level to the other. The **crossing
angle** is the rotation at which the first wheel reaches the step edge.

- Best crossing angle achieved: **52.6 degrees** (WB=12, TW=8)
- The robot needs **90 degrees** of rotation
- This means wheels MUST climb the 3.5" effective step mid-turn

### 2. Step climbing kills momentum
Even with a 1" chamfer reducing the effective step to 3.5", the wheel (R=3.5") must climb
a step equal to its own radius. This requires:
- Enormous instantaneous torque (the step acts as a wall)
- The robot must maintain rotational momentum while one or more wheels are airborne
- When wheels do climb, the chassis tilts violently (85-116 degrees = nearly flipping over)

### 3. Only one extremely narrow configuration works (barely)
The 3 passing configs ALL share:
- **R=3.5"** (smallest wheel - only one that can fit on inner floor)
- **W=2.0"** (narrowest tire)
- **TW=8"** (narrowest track width tested)
- The best one still tilts **85 degrees** (nearly perpendicular to the floor)
- Radial drift of **7-10 inches** (robot doesn't stay centered on the step)

### 4. Sensitivity analysis shows no design margin
- Increasing wheel radius to 4.0" or 4.5": **all fail** (even at TW=8)
- Increasing track width to 10" or 12": **all fail** (even at R=3.5)
- Increasing wheelbase to 12": **fails** (highest crossing angle but can't climb)
- Doubling torque (50 to 100 Nm): marginal effect, only helps 1 of 3 configs

### 5. The "successful" turns are dangerous
- **Max tilt 85-116 degrees**: robot nearly flips upside down during the maneuver
- **Drift 7-10 inches**: robot slides radially, risking contact with center post or outer wall
- In a real reactor with delicate plasma-facing components, this behavior is unacceptable

---

## Why Mecanum Wheels Solve This

| Problem                          | Skid-Steer              | Mecanum                        |
|----------------------------------|-------------------------|--------------------------------|
| Point turn mechanism             | Wheels scrub sideways   | Rollers allow lateral slip     |
| Step crossing during turn        | Unavoidable             | Can translate laterally first  |
| Minimum turn radius              | Zero (but destructive)  | Zero (smooth, controlled)      |
| Tilt during maneuver             | 85-116 degrees          | Minimal (no step climbing)     |
| Radial drift                     | 7-10 inches             | Controllable                   |
| Requires step climbing           | Yes                     | No (can reposition first)      |
| Omnidirectional movement         | No                      | Yes                            |
| Motion while straddling step     | Only forward/back       | Any direction                  |

With mecanum wheels, the robot can:
1. Land straddling the step (radial orientation) -- same as now
2. **Translate laterally** (sideways) without rotating -- no step crossing needed
3. Rotate in place on flat ground if needed
4. Combine translation and rotation for smooth, controlled repositioning
5. Never need to climb the step during a turn

---

## Data Files

- **Raw results (JSON)**: `step_sweep.json` -- 60 trial results with all parameters and metrics
- **Sweep script**: `../step_sweep.py` -- Isaac Sim parametric sweep (reproduces these results)
- **Calculator**: `../turn_calculator.py` -- Standalone analytical feasibility checker (no Isaac Sim needed)

---

## Test Parameters Reference

```
Simulation:     Isaac Sim 5.1.0, PhysX, 60 Hz
Robot mass:     40 lbs (18.1 kg) + 4x 2.5 lb wheels
Drive:          Skid-steer, velocity-controlled, 15 rad/s
Friction:       mu_s=0.8, mu_d=0.6 (generous rubber-on-graphite)
Step wall:      36 box segments + 36 chamfer ramps at r=55.5"
Trial timeout:  20 seconds
Success:        >= 90 degrees cumulative yaw rotation
```

*Generated 2026-03-09 from Isaac Sim Phase 2 straddle turn sweep (NVIDIA Isaac Sim 5.1.0)*
