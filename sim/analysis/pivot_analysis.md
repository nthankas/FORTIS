# FORTIS Pivot (Point Turn) Analysis

## Overview

The robot must perform zero-radius point turns inside the reactor while straddling the 4.5" step between the inner and outer floor. This analysis compares skid-steer pivot (impossible) vs X-drive pivot (viable).

## Reactor Step Geometry

```
         Center Post (r=38")
              |
   Inner floor: Z=-53.8"   r = 45.8" to 54.8"
   ─────────────────────────┐
                   Step wall │ 4.5" drop
   ─────────────────────────┘
   Outer floor: Z=-49.3"   r = 56.2" to 70.3"
```

- Step height: 4.5" (3.5" effective with 1" chamfer in skid-steer sim)
- Step radius: 55.5" from reactor center
- Step gap: 1.4" horizontal (r=54.8" to 56.2")

## X-Drive Pivot Mechanics

In an X-drive (4 omni wheels at 45-degree chamfered corners), a point turn is achieved by spinning all wheels at equal speed with alternating direction. The omni rollers allow each wheel to slide freely perpendicular to its drive axis, so **no wheel scrubs sideways**.

### Pivot Kinematics

For pure rotation at angular velocity omega, each wheel velocity is:

```
v_wheel = omega x r_wheel_from_center / WHEEL_RADIUS

For 15x9" chassis with 3" chamfers:
  Wheel positions: (±5.39", ±5.39") from chassis center
  Distance from center: 7.62" = 0.1936 m

At omega = 0.5 rad/s (typical rotate command):
  Wheel surface speed: 0.5 x 0.1936 = 0.097 m/s
  Wheel angular rate: 0.097 / 0.1016 = 0.95 rad/s = 9.1 RPM
```

### Torque During Pivot on Step

When the robot straddles the step with 17-degree tilt, pivot torque has two components:
1. **Rotational inertia**: Negligible at low speeds (< 0.5 rad/s)
2. **Friction asymmetry**: Inner wheels (lower floor) carry more weight due to tilt

```
Tilt angle:     17 degrees
Weight:         200.1 N
Normal force split (approximate):
  Inner wheels: 200.1 x cos(17) x (1 + sin(17)/2) / 2 = ~55 N each
  Outer wheels: 200.1 x cos(17) x (1 - sin(17)/2) / 2 = ~40 N each

Friction torque per wheel = mu x N x r_wheel_from_center
  Inner: 0.5 x 55 x 0.1936 = 5.3 Nm
  Outer: 0.5 x 40 x 0.1936 = 3.9 Nm
  Peak per wheel: 5.3 Nm
```

### Margin

| Gear Ratio | Available | Peak Demand | Margin |
|------------|-----------|-------------|--------|
| 5:1 | 18.75 Nm | 5.3 Nm | **3.5x** |
| 9:1 | 33.75 Nm | 5.3 Nm | **6.4x** |

## Why X-Drive Pivot Works Where Skid-Steer Fails

### Skid-Steer Pivot Failure Mode

In skid-steer, a point turn requires wheels to scrub laterally. When straddling the step:
- Wheels trace circular arcs around the robot center
- Those arcs cross the step edge at a calculable "crossing angle"
- **Best crossing angle achieved**: 52.6 degrees — short of the required 90 degrees
- When wheels reach the step edge, they must climb a 3.5" wall mid-turn
- Result: 85-116 degree tilt (near flip), 7-10" radial drift, 95% failure rate

### X-Drive Pivot Advantage

| Factor | Skid-Steer | X-Drive |
|--------|-----------|---------|
| Wheels cross step during pivot | Yes (unavoidable) | **No** (rollers allow free lateral slip) |
| Step climbing required | Yes | **No** |
| Peak chassis tilt during pivot | 85-116 deg | **17 deg** (step tilt only) |
| Radial drift during pivot | 7-10" | **< 1"** (controlled) |
| Pass rate (90 deg turn) | 5% (3/60) | **100%** |
| Torque demand | Unbounded (step impact) | 5.3 Nm (friction only) |

## Simulation Observations

From Isaac Sim testing on flat ground (`xdrive_o3dyn.py`):
- Robot pivots smoothly at 0.3-0.5 rad/s with no instability
- Yaw tracking error: +/-4 degrees
- No lateral drift during pure rotation
- Torque peaks during rotation: 2-4 Nm (below step-straddle estimate due to symmetric loading on flat ground)

From reactor testing (`xdrive_reactor_v2.py`):
- Pivot on step produces asymmetric wheel loading due to 17-degree tilt
- Robot maintains position (< 1" drift) during pivot
- Belly rollers do not contact ground during pivot (clearance = 4.9" on flat side)
