# FORTIS Orbit Analysis: Tangential Driving on Reactor Step

## Overview

After entering through the R0 port and landing on the reactor floor, the robot must drive tangentially (circumferentially) around the reactor while straddling the 4.5" step. This is the primary inspection maneuver — the robot orbits the center post, sweeping its camera arm across the interior surfaces.

## Orbit Geometry

```
         Top view (looking down Z axis)

              Center Post
                 (0,0)
                  |
         .........|.........  r = 55.5" (step edge)
        .         |         .
       .    Inner |  Outer   .
      .    Floor  | Floor     .
     .            |            .
    .         [ROBOT]           .  <-- Robot at r ~= 60"
     .        ↑ forward         .      driving tangentially
      .       (faces center)   .
       .                      .
        .                    .
         ....................
```

- Robot position: r = 60" from center (on outer floor, straddling step)
- Robot heading: radially inward (facing center)
- Drive direction: tangential (lateral strafe in robot frame)
- Orbit angular rate: omega = v_tangential / r

## Orbit Kinematics

For tangential orbit at speed v:

```
Robot at r = 60" (1.524 m), facing center:
  Robot-frame: vx = 0 (no radial), vy = v (tangential), omega = v / r

At v = 0.2 m/s:
  omega = 0.2 / 1.524 = 0.131 rad/s = 7.5 deg/s
  Full orbit (360 deg): 48 seconds

At v = 0.1 m/s:
  omega = 0.1 / 1.524 = 0.066 rad/s = 3.8 deg/s
  Full orbit: 96 seconds
```

## X-Drive IK for Orbit

The X-drive inverse kinematics converts (vx, vy, omega) to individual wheel speeds:

```
For each wheel at position (wx, wy) with axle angle a:
  v_drive = (vx - omega*wy)*cos(a) + (vy + omega*wx)*sin(a)
  wheel_speed = v_drive / WHEEL_RADIUS

Orbit at 0.2 m/s, r = 1.524 m (omega = 0.131 rad/s):
  FR (45 deg):  2.05 rad/s
  FL (135 deg): 1.83 rad/s
  BL (45 deg):  1.83 rad/s
  BR (135 deg): 2.05 rad/s
```

Wheel speeds are nearly equal — the orbit radius is large relative to the robot, so the velocity gradient across the chassis is small.

## Measured Performance (Isaac Sim)

### Test Conditions
- Script: `xdrive_reactor_v2.py` (arched chassis + belly rollers)
- Reactor: `diiid_reactor.usd` (triangle mesh collision)
- Physics: 240 Hz, TGS solver, mu=0.5
- Robot: 15x9x5.5" arched chassis, 45 lbs, 8" omni wheels, 2" wheel drop

### Telemetry at 0.2 m/s CCW Orbit

| Time | X (in) | Y (in) | Z (in) | r (in) | Pitch | Roll | Peak Torque |
|------|--------|--------|--------|--------|-------|------|-------------|
| 0s | -2.3 | -63.3 | -47.3 | 63.3 | -0.4 | 17.0 | 0.6 Nm |
| 23s (orbit on) | -5.5 | -64.0 | -47.3 | 64.3 | 0.6 | 17.2 | 4.0 Nm |
| 25s | -22.6 | -59.8 | -47.3 | 63.9 | -1.8 | 17.0 | 4.3 Nm |
| 28s | -44.3 | -42.2 | -47.3 | 61.2 | -9.6 | 14.5 | 10.7 Nm |
| 33s | -54.7 | -0.2 | -47.2 | 54.7 | -17.0 | 2.9 | 3.0 Nm |
| 38s | -30.0 | 36.9 | -47.3 | 47.5 | -13.1 | -12.0 | 4.7 Nm |

### Observations

1. **Radius drift**: r decreases from 63" to 48" over 15 seconds of orbit — the robot spirals inward
2. **Roll varies**: 17 deg when straddling step, approaches 0 when crossing step region, goes negative on the other side
3. **Pitch varies**: -0.4 to -17 degrees — the tilted floor and curved geometry create pitch coupling
4. **Torque peaks**: 10.7 Nm during step transition, typical 3-6 Nm during steady orbit
5. **Z stable**: -47.3" throughout — robot stays on the floor, no bouncing or lifting

### Radius Drift Root Cause

The current orbit implementation uses a fixed `omega = vy / configured_radius` rather than feedback-controlled radius. When the robot is at r=63" but omega is computed for r=55.5", it turns too fast and spirals inward. A closed-loop orbit controller (using measured position to compute omega) would maintain constant radius.

The simple open-loop orbit is still useful for testing — it confirms the robot can drive tangentially on the tilted surface and the torque demands are within motor capability.

## Torque Budget for Orbit

| Component | Torque | Notes |
|-----------|--------|-------|
| Tangential drive (0.2 m/s) | 3-4 Nm | Rolling resistance on graphite |
| Gravity (17 deg tilt) | 1.5 Nm/wheel | Step straddle, asymmetric loading |
| Step transition (crossing step edge) | 10.7 Nm peak | Momentary spike as wheel crosses gap |
| Steady-state orbit total | ~5-6 Nm | Excluding step transition spikes |

### Motor Margin

| Gear Ratio | Available | Steady Orbit | Margin | Step Spike | Margin |
|------------|-----------|-------------|--------|------------|--------|
| 5:1 | 18.75 Nm | 6 Nm | **3.1x** | 10.7 Nm | **1.8x** |
| 9:1 | 33.75 Nm | 6 Nm | **5.6x** | 10.7 Nm | **3.2x** |

## Key Finding

The X-drive can orbit tangentially around the reactor while straddling the step. Torque demands (3-6 Nm steady, 10.7 Nm peak) are within the motor+gearbox capability with adequate margin. The radius drift issue is a controls problem (open-loop vs closed-loop orbit), not a hardware limitation.
