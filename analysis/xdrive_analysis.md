# FORTIS X-Drive Drivetrain Analysis

## Configuration

### Drivetrain Layout
Four omni wheels mounted at the 45-degree chamfered corners of an octagonal chassis, forming an X-drive (also called holonomic drive). Each wheel's drive axis is perpendicular to the chassis diagonal at its corner.

```
        Front (+X)
     FL ╲       ╱ FR
        ╲ ┌───┐ ╱
         ─┤   ├─
         ─┤   ├─        Wheels at 45-deg corners
        ╱ └───┘ ╲       Drive axes shown
     BL ╱       ╲ BR
        Back (-X)
```

### Robot Specifications

| Parameter | Value |
|-----------|-------|
| Chassis | 15" x 9" x 5.5" octagonal prism |
| Chamfers | 3" face (2.12" cut depth) at all 4 corners |
| Mass | 40 lbs (18.144 kg) total: chassis body + arm lumped 14.144 kg + 4 x 1.0 kg wheels |
| Wheels | AndyMark 8" Dualie Plastic Omni (am-0463) |
| Wheel diameter | 203 mm (8.0") |
| Wheel width | 2.04" (52 mm) |
| Wheel mass | 2.2 lbs (1.0 kg) each |
| Roller count | 36 rollers per wheel, 80A durometer |
| Motors | REV NEO 2.0 Brushless (x4) |
| Gearbox | REV MAXPlanetary (5:1 or 9:1) |
| Wheel drop | 2.0" (wheels extend below motor mount plane) |

### Simulation Model

| Parameter | Value |
|-----------|-------|
| Simulator | NVIDIA Isaac Sim 5.1.0 (PhysX) |
| Physics rate | 240 Hz |
| Solver | TGS with CCD and stabilization |
| Wheel model | Sphere-approximated rollers: 2 rows x 12 rollers x 3 spheres = 72 spheres/wheel |
| Sphere radius | 12.7 mm |
| Friction (graphite) | mu_s = mu_d = 0.5 |
| Collision | convexDecomposition (chassis), sphere (wheels) |

## Inverse Kinematics

The X-drive IK converts desired body velocity (vx, vy, omega) to individual wheel angular velocities:

```
For wheel i at position (wx_i, wy_i) with axle angle a_i:
  v_drive_i = (vx - omega * wy_i) * cos(a_i) + (vy + omega * wx_i) * sin(a_i)
  wheel_speed_i = v_drive_i / WHEEL_RADIUS
```

| Wheel | Position (in) | Axle Angle |
|-------|--------------|------------|
| FR | (+5.39, -5.39) | 45 deg |
| FL | (+5.39, +5.39) | 135 deg |
| BL | (-5.39, +5.39) | 45 deg |
| BR | (-5.39, -5.39) | 135 deg |

### Motion Capabilities

| Motion | vx | vy | omega | Result |
|--------|----|----|-------|--------|
| Forward | + | 0 | 0 | Translate along X axis |
| Strafe | 0 | + | 0 | Translate along Y axis |
| Diagonal | + | + | 0 | Translate at 45 degrees |
| Pivot | 0 | 0 | + | Rotate in place |
| Orbit | 0 | + | +v/r | Circular path around center |
| Any combination | vx | vy | omega | Full holonomic motion |

## Measured Performance

### Flat Ground (xdrive_o3dyn.py)

| Metric | Forward 0.2 m/s | Strafe 0.2 m/s | Diagonal 0.2 m/s |
|--------|-----------------|----------------|-------------------|
| Lateral drift | 3.1% | ~3% | ~4% |
| Yaw oscillation | +/-4 deg | +/-3 deg | +/-5 deg |
| Peak torque (sim) | 6.1 Nm | 5.5 Nm | 6.8 Nm |
| Estimated real torque | ~5.1 Nm | ~4.6 Nm | ~5.7 Nm |
| Stable at rest | Yes | Yes | Yes |

Note: Simulation overestimates torque by ~20% because sphere-contact rollers have higher friction than real free-spinning rollers.

### Reactor Floor (xdrive_reactor_v2.py)

| Metric | Value |
|--------|-------|
| Step straddle tilt | 17 degrees |
| Static hold torque | 0.5-0.6 Nm |
| Forward 0.2 m/s on step | 3-4 Nm typical |
| Orbit 0.2 m/s tangential | 3-6 Nm typical, 10.7 Nm peak at step transition |
| Belly roller contact during orbit | 0 (rollers above ground, only engage at R0 lip) |
| Radial drift during orbit | ~1"/s (open-loop, correctable with feedback) |

## Underbelly Clearance Profile

With 2" wheel drop and 5.5" chassis height:

```
                    Ground level (Z=0)
                    ──────────────────

Motor mount flat:   3.25" above ground
Arch ramp:          3.25" to 5.75" (linear taper)
Arch ceiling:       5.75" above ground
Roller bottom:      4.88" above ground (0.25" protrusion below arch)
Wheel contact:      0" (ground level)

Cross-section (front view):
         ┌────────────────────┐  ← top (5.75" + wheel drop + radius = 8.75")
         │    ╱──────────╲    │  ← arch ceiling (5.75")
         │   ╱  rollers   ╲   │  ← roller bottom (4.88")
         │  ╱              ╲  │
         │ │ motor    motor │ │  ← motor mount (3.25")
         └─┤               ├─┘
           ◯               ◯    ← wheels (ground contact)
    ─────────────────────────────  ground
```

## Tunnel Fit

| Orientation | Dimension | Tunnel (14.75" usable) | Fit? |
|-------------|-----------|----------------------|------|
| Normal (wheels down) | 15" L x 11.2" W | Too long | No |
| On side (90 deg roll) | 9" W x ~11" H | 9" x 11" | Yes |
| Stowed with folded arm | TBD | TBD | TBD |

The robot enters the tunnel on its side (9" face first), then is rotated upright after passing through.

## Why X-Drive Over Other Drivetrains

| Factor | Skid-Steer | Mecanum | X-Drive (Omni) |
|--------|-----------|---------|----------------|
| Holonomic motion | No | Yes | Yes |
| Pivot on step | Fails (95%) | Works | Works |
| Step crossing required | Yes (during turns) | No | No |
| Wheel count | 4 | 4 | 4 |
| Roller complexity | None | Angled (45 deg) | Straight (90 deg) |
| Force in drive direction | 100% | ~71% (cos 45) | ~71% (cos 45) |
| Lateral force | 0% (scrub) | ~71% | ~71% |
| Wheel availability (COTS) | Abundant | Limited at 8" | Available (AndyMark) |
| Packaging at corners | Tight | Tight | Natural (45-deg chamfer) |

X-drive was selected over mecanum because:
1. 90-degree rollers are mechanically simpler and more robust than 45-degree mecanum rollers
2. COTS 8" omni wheels are readily available (AndyMark Dualie)
3. The 45-degree corner mounting naturally fits the octagonal chassis chamfers
4. Kinematic performance is equivalent to mecanum for this application

## Key Findings

1. **X-drive provides full holonomic motion** — forward, strafe, diagonal, pivot, orbit, and any combination
2. **3.1% drift at 0.2 m/s** — acceptable for a teleoperated inspection robot with visual feedback
3. **Peak torque ~5 Nm** on flat ground, **~10.7 Nm** during step transitions — within motor capability (18.75 Nm at 5:1, 33.75 Nm at 9:1)
4. **17-degree tilt** when straddling the step is stable and drivable
5. **Belly rollers** (passive, in arch zone) do not interfere with normal driving — they only engage during R0 port lip crossing
6. **Open-loop orbit drifts inward** — a closed-loop radius controller is needed for sustained circumferential driving
