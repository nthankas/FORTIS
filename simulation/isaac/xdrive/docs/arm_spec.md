# FORTIS Arm Mass Properties — Sim3 Setup

## Purpose
Mass, CG, and inertia data for Isaac Sim Sim3: chassis stability with arm poses.
Build arm from USD primitives (not CAD imports). Place rigid body at each joint with correct mass.

## Coordinate Frame
- Origin: J1 base rotation axis (top of chassis, center or offset rearward TBD)
- +X: forward (toward center post during operation)
- +Z: up
- Arm extends in XZ plane when J1 = 0°

## Joint Geometry

| Joint | Type | Axis | Distance from previous joint |
|-------|------|------|----------------------------|
| J1 | revolute | Z (yaw) | 0 (on chassis top) |
| J2 | revolute | Y (pitch) | 50mm above J1 (motor+gearbox stack) |
| J3 | revolute | Y (pitch) | 558.8mm from J2 (22") |
| J4 | revolute | Y (pitch) | 482.6mm from J3 (19") |
| EE | fixed | — | 203.2mm from J4 (8") |

Total reach from J1 to end effector tip: 1294.6mm (50.97")

## Component Masses — CONFIRMED

### Motors

| Part | Joint | Mass | Dimensions | Torque | Current | Source |
|------|-------|------|-----------|--------|---------|--------|
| NEMA 23 23HS22-2804S | J1, J2 | 700g each | 57×57×56mm | 1.24 Nm hold | 2.8A/phase | StepperOnline datasheet |
| NEMA 17 17HS24-2104S | J3, J4 | 500g each | 42×42×60mm | 0.65 Nm hold | 2.1A/phase | StepperOnline datasheet |

### Gearboxes

| Part | Joint | Mass | Dimensions | Ratio | Backlash | Efficiency | Rated torque | Price | Source |
|------|-------|------|-----------|-------|----------|------------|-------------|-------|--------|
| Cricket Drive MK II | J3, J4 | 80g each | 42×42×26mm | 25:1 | ±8.5 arcmin | 85.5% | 11-12 Nm | $54.95 | Sweep Dynamics spec sheet |
| TBD NEMA 23 cycloidal/planetary | J1, J2 | ~500g each (estimate) | ~57×57×40mm (estimate) | ~20-25:1 | TBD | TBD | TBD | TBD | NOT YET SELECTED |

**CRITICAL NOTE:** Cricket Drive MK II fits NEMA 17 bolt pattern (42×42mm) ONLY. It does NOT fit NEMA 23 (57×57mm). J1 and J2 need a separate gearbox solution. Options being evaluated:
- StepperOnline NEMA 23 planetary (various ratios, ~$55-90, ~500-1000g, Chinese mfg)
- Custom cycloidal for NEMA 23 (not commercially available off-shelf in this size from US vendors)
- Downsize J1/J2 to NEMA 17 if torque analysis shows NEMA 17 + 25:1 Cricket Drive is sufficient

### Camera

| Part | Location | Mass | Dimensions | Source |
|------|----------|------|-----------|--------|
| Orbbec Gemini 2 | End effector | 445g | ~165×40×50mm | Orbbec spec sheet |

### Links — Carbon Fiber Option

| Link | Length | Tube spec | Mass | Source |
|------|--------|-----------|------|--------|
| J2→J3 | 558.8mm (22") | 1.0" OD × 0.060" wall CF | 45g | Calculated, ρ=1.55 g/cm³ |
| J3→J4 | 482.6mm (19") | 0.75" OD × 0.050" wall CF | 28g | Calculated |
| J4→EE | 203.2mm (8") | 0.625" OD × 0.050" wall CF | 9g | Calculated |
| **Total links** | | | **82g** | |

### Links — 6061-T6 Aluminum Option (for comparison)

| Link | Length | Tube spec | Mass |
|------|--------|-----------|------|
| J2→J3 | 558.8mm (22") | 1.0" OD × 0.065" wall Al | 110g |
| J3→J4 | 482.6mm (19") | 0.75" OD × 0.065" wall Al | 68g |
| J4→EE | 203.2mm (8") | 0.625" OD × 0.065" wall Al | 24g |
| **Total links** | | | **202g** |

### Miscellaneous

| Item | Mass | Notes |
|------|------|-------|
| Brackets/couplers/fasteners | 120g | 4 joints × ~30g each |
| Wiring harness | 80g | ~4m of motor cables + USB 3.0 for Gemini |
| **Total misc** | **200g** | Distributed across joints |

## Lumped Mass Model for Isaac Sim

Use these as rigid body masses at each joint position. Each body = motor + gearbox + half of adjacent link spans + share of misc.

### With Carbon Fiber links, NEMA 23 planetary at 500g estimate:

| Body | Mass | Distance from J1 | Tipping moment (horizontal) | What's in it |
|------|------|------------------|----------------------------|-------------|
| J1_base | 1250g | 0mm | 0 Nm | NEMA 23 + J1 gearbox(500g) + 50g misc |
| J2_shoulder | 1272g | 50mm | 0.62 Nm | NEMA 23 + J2 gearbox(500g) + ½ J2-J3 link(22g) + 50g misc |
| J3_elbow | 666g | 559mm | 3.65 Nm | NEMA 17 + Cricket MK II(80g) + ½ links(36g) + 50g misc |
| J4_wrist | 648g | 1041mm | 6.63 Nm | NEMA 17 + Cricket MK II(80g) + ½ links(19g) + 50g misc |
| EE_camera | 450g | 1245mm | 5.49 Nm | Gemini 2(445g) + ½ J4-EE link(5g) |
| **TOTAL** | **4286g (9.45 lbs)** | | **16.39 Nm** | |

Arm CG from J1 (horizontal): 390mm (15.3")

### With Carbon Fiber links, Cricket MK II on ALL joints (if J1/J2 downsize to NEMA 17):

| Body | Mass | Distance from J1 | Tipping moment (horizontal) |
|------|------|------------------|----------------------------|
| J1_base | 630g | 0mm | 0 Nm |
| J2_shoulder | 652g | 50mm | 0.32 Nm |
| J3_elbow | 666g | 559mm | 3.65 Nm |
| J4_wrist | 648g | 1041mm | 6.63 Nm |
| EE_camera | 450g | 1245mm | 5.49 Nm |
| **TOTAL** | **3046g (6.71 lbs)** | | **16.09 Nm** |

Note: All-NEMA-17 arm saves 1.24 kg (2.7 lbs) and simplifies to one motor type + one gearbox type. Tipping moment barely changes because J1/J2 are near the base (small moment arm). The weight savings are almost entirely at the base where they don't affect tipping. The question is whether NEMA 17 + Cricket 25:1 provides enough torque for J1/J2.

### J1/J2 Torque Check (NEMA 17 + Cricket 25:1)

- NEMA 17 holding torque: 0.65 Nm
- Cricket 25:1 at 85.5% efficiency: 0.65 × 25 × 0.855 = **13.9 Nm at joint output**
- Cricket rated torque: 11-12 Nm (so we'd be slightly over spec at full holding torque)
- Worst-case tipping moment the arm exerts on J2: ~16 Nm (fully horizontal)
- **13.9 Nm < 16 Nm → NEMA 17 on J2 CANNOT hold the arm horizontal**
- At 45° below horizontal: 16 × cos(45°) = 11.3 Nm → borderline
- At 60° below horizontal: 16 × cos(60°) = 8.0 Nm → OK with margin

**Conclusion:** NEMA 17 + Cricket MK II works for J3/J4 easily and possibly J1, but J2 (shoulder) needs NEMA 23 or a higher-torque solution to hold the arm in upper poses. Keep NEMA 23 on J2 minimum.

## Chassis Tipping Analysis

### Parameters

- Total robot mass: 40 lbs (18.14 kg) nominal
- Arm mass (NEMA 23 J1/J2 variant): 4.29 kg (9.45 lbs)
- Chassis-without-arm mass: 13.85 kg (30.5 lbs)
- Front wheel offset from chassis center: ~4.5" (114mm) — tipping edge
- Side wheel offset: ~3.0" (76mm)

### Restoring moment

- Chassis CG at center, wheels 114mm forward = 13.85 × 9.81 × 0.114 = **15.49 Nm**

### Tipping moments by arm pose

| Arm pose | Moment fraction | Tipping moment | Safety factor | Status |
|----------|----------------|----------------|---------------|--------|
| Fully horizontal forward | 1.00× | 16.39 Nm | 0.95× | **TIPS** |
| 30° below horizontal | 0.87× | 14.22 Nm | 1.09× | Marginal |
| 45° below horizontal | 0.71× | 11.60 Nm | 1.34× | OK |
| 60° below horizontal | 0.50× | 8.20 Nm | 1.89× | Good |
| Straight down (stowed) | 0.00× | 0 Nm | ∞ | Perfect |

### Mitigation options (pick one or combine):

1. **Move J1 rearward 2-3" from chassis center** — adds 2-3" to restoring moment arm
2. **Software joint limits** — prevent arm from going above 30° below horizontal
3. **Add counterweight at rear** — 1-2 lbs at rear overhang
4. **Cable tension assists** — tether attachment at rear provides anti-tip force (not modeled)

## What to do in Isaac Sim (Sim3)

1. Place 5 rigid body primitives (boxes or spheres) at the joint positions listed above
2. Connect with revolute joints matching the axis column
3. Set mass on each body from the lumped mass table
4. Sweep J2 angle from -90° (straight down) to +30° (above horizontal) in 5° increments
5. At each J2 angle, sweep J3 from -90° to +90°
6. Log: which wheel loses contact first, at what pose angles
7. Generate stability envelope plot: J2 angle vs J3 angle, colored by min wheel normal force
8. Overlay the inspection reach envelope to confirm usable poses are within stable region

## Open Items

- [ ] Get actual NEMA 23 gearbox selection (cycloidal or planetary, mass, dimensions, price)
- [ ] Confirm Cricket Drive MK II availability and lead time from Sweep Dynamics
- [ ] Decide J1 motor: NEMA 23 or NEMA 17 (J1 is base rotation, sees less gravity load than J2)
- [ ] Decide J1 position offset on chassis (centered vs rear-biased)
- [ ] Get CF tube quotes from RockWest or DragonPlate with published modulus data
- [ ] Add AS5048A encoder masses (~5g each, negligible but include for completeness)
