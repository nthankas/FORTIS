# FORTIS Arm Mass Properties — 4-DOF Parallel Link (36" reach)

## Purpose
Mass, CG, and inertia data for Isaac Sim: chassis + arm torque analysis.
Build arm from USD primitives (not CAD imports). Place rigid body at each
joint with correct mass.

## Coordinate Frame
- Origin: J1 base rotation axis (top of chassis, back edge, centerline)
- +X: forward (toward center post during operation)
- +Z: up
- Arm extends in XZ plane when J1 = 0 deg

## Joint Geometry

| Joint | Type | Axis | Distance from previous joint |
|-------|------|------|----------------------------|
| J1 | revolute | Z (yaw) | 0 (on chassis top, back edge) |
| J2 | revolute | Y (pitch) | 86mm above J1 (motor+gearbox stack) |
| J3 | revolute | Y (pitch) | 431.8mm from J2 (17") |
| J4 | revolute | Y (pitch) | 381.0mm from J3 (15") |
| EE | fixed | -- | 101.6mm from J4 (4", gripper at tip) |

Total reach from J2 to EE tip: 914.4mm (36.0")

## Stowed Pose (flat zigzag)
Links alternate direction when stowed flat on chassis top:
- L2: +X (forward, 17")
- L3: -X (backward, 15")
- L4: +X (forward, 4", gripper at tip)

Each link side-by-side in Y (parallel layout), all at same Z.

## Hardware

### Motors
| Part | Qty | Mass | Torque | Source |
|------|-----|------|--------|--------|
| NEMA 17 stepper 0.80Nm | 4 | ~580g each | 0.80 Nm hold | Lin Engineering / Oriental Motor |

### Gearboxes
| Part | Qty | Mass | Ratio | Rated torque | Source |
|------|-----|------|-------|-------------|--------|
| Cricket Drive MK II | 4 | 80g each | 25:1 | 11-12 Nm | Sweep Dynamics |

### Drivers
| Part | Qty | Source |
|------|-----|--------|
| TMC5160-TA IC + passives | 4 | DigiKey (Analog Devices) |

### Depth Camera
| Part | Location | Mass | Source |
|------|----------|------|--------|
| Orbbec Gemini 2 | J2 shoulder, 2" from J2 along L2 | 98g | Orbbec spec sheet |

Mounted on J2 body looking down the arm — rotates with J1 yaw and J2 pitch,
always has line of sight to elbow/wrist/gripper and target. ~30" from gripper
when extended (well within depth camera effective range, not too close).

### Gripper
| Part | Location | Mass | Source |
|------|----------|------|--------|
| Gripper assembly | L4 tip (EE) | 500g | User-specified |

### Links — Carbon Fiber Square Tube
| Spec | Value |
|------|-------|
| Shape | Square, 0.79" x 0.79" (20.07mm) |
| Composition | Solid CF (Woven and Uni Directional) |
| Fiber modulus | Standard (33 Msi) |
| Linear density | 0.0053 lb/in (STD) / 0.0088 lb/in (UNI3) |
| Source | RockWest Composites (UT) |

| Link | Length | Mass (STD density) |
|------|--------|--------------------|
| L2 (J2->J3) | 17" (431.8mm) | 40.9g |
| L3 (J3->J4) | 15" (381.0mm) | 36.1g |
| L4 (J4->EE) | 4" (101.6mm) | 9.6g |
| **Total links** | | **86.6g** |

## Lumped Mass Model for Isaac Sim

Each articulation body = joint mass + link mass + extras.
Joint mass = 0.629 kg (motor 0.58 kg + hardware 49g).

| Body | Contents | Mass (kg) | COM (body-local) |
|------|----------|-----------|-------------------|
| J1_base | J1 motor stack | 0.629 | (0, 0, +43mm) along +Z |
| J2_shoulder | J2 joint + L2 link + camera | 0.768 | weighted avg of joint@origin + link midpoint@+8.5" + camera@+2" |
| J3_elbow | J3 joint + L3 link | 0.665 | weighted avg of joint@origin + link midpoint@-7.5" |
| J4_wrist | J4 joint + L4 link + gripper | 0.689 | joint@origin + link midpoint@+2" + gripper@+4" |
| **Total arm** | | **2.751 kg (6.06 lb)** | |

## Arm Mount
- Position: chassis back edge, centerline, top surface
- X = -CHASSIS_L/2 = -7.68" (in chassis frame)
- Y = 0
- Z = +CHASSIS_H/2 = +3.55" (chassis top)
- J1 stack height: 86mm, putting J2 pivot at Z = +6.95" in chassis frame

## Key Specs

| Spec | Value |
|------|-------|
| Total arm mass | ~2.75 kg (6.06 lb) |
| Reach | 36" (17+15+4) |
| Links | CF square tube 0.79"x0.79", pultruded |
| Joints | 4 (J1 yaw + J2-J4 pitch) |
| Joint mass | 0.629 kg each |
| Gripper mass | 500g |
| Camera mass | 98g (Orbbec Gemini 2 on J2) |
| Bus voltage | 24V |

## Open Items

- [ ] Verify arm geometry and masses in Isaac Sim GUI (user testing)
- [ ] Run torque sweep: J2 angle vs required torque at each joint
- [ ] Confirm Cricket Drive MK II availability from Sweep Dynamics
- [ ] Decide J1 position offset (centered vs rear-biased, currently back edge)
- [ ] Get CF tube quotes from RockWest with published modulus data
- [ ] Select specific gripper servo (SG90, XL330-M077, etc.)
