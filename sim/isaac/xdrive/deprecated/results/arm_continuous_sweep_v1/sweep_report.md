# Arm Torque Sweep Report

Physics-validated torque sweep of the FORTIS 4-DOF arm across all reachable joint configurations. The robot is on the 4.5" step environment (deployment scenario) for all primary results.

Three arm lengths (36", 30", 24") were tested bare and with a 3 lb payload at the end effector, giving **6 configurations**. Each sweeps **2574 poses** (J2 in 15° steps, J3 in 20° steps, J4 in 30° steps) with J1 fixed at 0°.

---

## How It Works

The arm is simulated in Isaac Sim with collision **disabled** on all arm links. This lets every joint combination be reached regardless of geometry. For each pose the arm is teleported to the target angles, a PD controller holds position for ~0.4 s, then joint torques are measured over ~0.1 s.

Because collision is off, many of the recorded poses are physically impossible (arm through the chassis, through itself, underground, etc.). A post-processing filter removes these using geometry checks. The result is two datasets per configuration: **raw** (every pose) and **filtered** (only physically realizable poses).

All 6 step-environment configurations achieved **2574/2574 poses converged, 0 NaN**. Physics-measured torques match independent base MOI (1/3ML^2 etc) and analytical gravity calculations to within **0.5%**.

---

## Arm Configuration

| Parameter | Value |
|-----------|-------|
| Joint mass (J3, J4) | 629 g each |
| Gripper mass | 500 g |
| Camera mass | 98 g (Orbbec Gemini 2, mounted on L1) |
| Link tube | 0.79" x 0.79" pultruded CF square tube |
| Loaded payload | 3 lb (1360 g) added at end effector |

The camera is fixed to L1, so it primarily affects J2 torque. The gripper sits at the tip of the arm and dominates J3/J4 loading. Adding the 3 lb payload roughly doubles peak torques across all joints.

---

## Filtered Results (Valid Poses Only)

Poses that pass all geometry and stability heuristic filters. These are the torques for physically realizable arm configurations on the step environment.

| Config | Valid Poses | J2 Peak (Nm) | J3 Peak (Nm) | J4 Peak (Nm) | J2 Avg (Nm) | J3 Avg (Nm) | J4 Avg (Nm) |
|--------|:-----------:|:------------:|:------------:|:------------:|:-----------:|:-----------:|:-----------:|
| **24" bare**   | 1845 / 2574 |  8.50 |  3.08 | 0.25 | 3.54 | 1.88 | 0.16 |
| **24" loaded** | 1845 / 2574 | 16.56 |  7.09 | 0.93 | 6.59 | 4.11 | 0.59 |
| **30" bare**   | 1890 / 2574 | 10.58 |  3.78 | 0.38 | 4.50 | 2.26 | 0.24 |
| **30" loaded** | 1867 / 2574 | 19.30 |  8.80 | 1.40 | 8.32 | 4.92 | 0.89 |
| **36" bare**   | 1880 / 2574 | 12.53 |  4.77 | 0.50 | 5.22 | 2.84 | 0.32 |
| **36" loaded** | 1694 / 2574 | 19.42 | 11.09 | 1.87 | 8.47 | 5.89 | 1.19 |

Peak torques are the worst-case values across all valid poses. These are the numbers motors must be sized for. Average torques are relevant for power and thermal sizing.

---

## Motor Sizing

Design-against numbers from the filtered step-environment data.

| Config | J2 Peak | J3 Peak | J4 Peak |
|--------|:-------:|:-------:|:-------:|
| **24" bare**   |  8.5 Nm |  3.1 Nm | 0.3 Nm |
| **24" loaded** | 16.6 Nm |  7.1 Nm | 0.9 Nm |
| **30" bare**   | 10.6 Nm |  3.8 Nm | 0.4 Nm |
| **30" loaded** | 19.3 Nm |  8.8 Nm | 1.4 Nm |
| **36" bare**   | 12.5 Nm |  4.8 Nm | 0.5 Nm |
| **36" loaded** | 19.4 Nm | 11.1 Nm | 1.9 Nm |

The 36" loaded configuration is the hardest case: J2 must hold ~19.4 Nm and J3 ~11.1 Nm. J4 requirements are modest across all configs (<2 Nm). We designed J3 and J4 to be accounting for the weight of an average NEMA17 + Cricket Drive MK II (keep in mind, that gearbox can only max output about 12nm), and it seems like bare or loaded - circkets will work for J3 and may not even be needed for J4. If the arm is to have to pick up something though, the cricket drive will not work for J2 and we will need a NEMA 23 and gearbox or NEMA 34 or gearbox. I think that is feasible for just a base line arm motor.

---

## Link Deflection Under Load

Using 25492 square CF tube (1.250" ID, 1.380" OD, 0.065" wall, E = 80.7 GPa, I = 2.42 x 10^-8 m^4). Link masses estimated from the tube's linear density (0.226 lb/ft = 0.336 kg/m) scaled to length. 

### Tip Load on L1 (Loaded Configs, Worst Case)

Everything outboard of J2 hangs off L1: J3 joint (629 g) + L2 mass + J4 joint (629 g) + L3 mass + end effector (gripper 500 g + payload 1360 g = 1860 g).

| Config | L2 Mass | L3 Mass | Total Tip Load |
|--------|:-------:|:-------:|:--------------:|
| 36" loaded | ~110 g | ~30 g | 3.26 kg (32.0 N) |
| 30" loaded | ~88 g  | ~22 g | 3.23 kg (31.7 N) |
| 24" loaded | ~73 g  | ~15 g | 3.21 kg (31.5 N) |

### L1 Deflection

| Config | Force (N) | Length (m) | Deflection (mm) |
|--------|:---------:|:----------:|:----------------:|
| 36" loaded | 32.0 | 0.432 | 0.44 |
| 30" loaded | 31.7 | 0.381 | 0.30 |
| 24" loaded | 31.5 | 0.254 | 0.09 |

### L2 Deflection

Outboard of J3: J4 joint (629 g) + L3 mass + end effector (1860 g).

| Config | Force (N) | Length (m) | Deflection (mm) |
|--------|:---------:|:----------:|:----------------:|
| 36" loaded | 24.5 | 0.381 | 0.23 |
| 30" loaded | 24.4 | 0.305 | 0.12 |
| 24" loaded | 24.3 | 0.305 | 0.12 |

### Total End-Effector Sag (Loaded, Stacked)

| Config | L1 | L2 | L3 | Total Sag |
|--------|:--:|:--:|:--:|:---------:|
| 36" loaded | 0.44 mm | 0.23 mm | ~0.02 mm | **~0.7 mm** |
| 30" loaded | 0.30 mm | 0.12 mm | ~0.01 mm | **~0.4 mm** |
| 24" loaded | 0.09 mm | 0.12 mm | ~0.01 mm | **~0.2 mm** |

Total sag is under 1 mm even in the worst case (36" loaded, arm fully extended). The CF tube is more than stiff enough

---

## Chassis Stability

Static tipping analysis: the combined center of gravity of the chassis (20.4 kg) plus all arm masses is projected onto the ground plane and checked against the wheel support polygon. J1 is swept from 0° to 345° in 15° steps; the worst-case J1 angle is reported.

**This assumes a level chassis.** The 4.5" step creates a permanent tilt that will shift the CG projection and reduce stability margins on the downhill side. Step tilt correction is a follow-up.

| Config | Poses Rejected | Worst Margin | Avg Margin | Tight Poses (<10 mm) |
|--------|:--------------:|:------------:|:----------:|:--------------------:|
| **24" bare**   |   0 (0.0%) | 43.0 mm / 1.69" | 74.2 mm | 0 |
| **24" loaded** |   0 (0.0%) | 11.4 mm / 0.45" | 45.7 mm | 0 |
| **30" bare**   |   0 (0.0%) | 33.8 mm / 1.33" | 63.0 mm | 0 |
| **30" loaded** |  23 (1.2%) |  0.5 mm / 0.02" | 35.5 mm | 139 (7.4%) |
| **36" bare**   |   0 (0.0%) | 25.2 mm / 0.99" | 54.7 mm | 0 |
| **36" loaded** | 186 (9.9%) |  0.0 mm / 0.00" | 28.4 mm | 151 (8.9%) |

The rejected poses are all fully-extended configurations with the arm reaching sideways (J1 ~ 90°), perpendicular to the long axis of the chassis. The chassis is narrower side-to-side (~9.4") than front-to-back (~15.4"), so sideways reach is the worst case for tipping.

The 24" arm is unconditionally stable at all poses. The 30" arm has a small number of marginal poses when loaded. The 36" loaded arm has ~10% of poses rejected for tipping, all at full extension.

---

## Filtering Details

A pose is rejected if **any** of the following checks fail:

**Chassis collision** -- 10 sample points along each link segment (L1, L2, L3). If any point falls inside the chassis bounding box (with 10 mm clearance margin), the pose is rejected. The box spans from the arm mount backward along the full chassis length.

**Self-collision (L3 vs L1)** -- Minimum distance between L3 and L1 segments, sampled at 12 points each. Rejected if closer than 15 mm (tube half-width + 5 mm clearance).

**Self-collision (L3 vs J1 base)** -- 8 sample points along L3 checked against the J1 base volume. Rejected if any point enters the base.

**Floor collision** -- Any joint position or link midpoint below the floor plane (with 10 mm + tube half-width margin).

**Tipping** -- CG projection outside the wheel support polygon at any J1 angle (see Chassis Stability above).

### Rejection Breakdown

| Arm | Loading | Total | Valid | Chassis | Self (L3-L1) | Self (L3-J1) | Floor | Tipping |
|-----|---------|:-----:|:-----:|:-------:|:------------:|:------------:|:-----:|:-------:|
| 24" | bare    | 2574 | 1845 (71.7%) | 487 | 100 | 0 | 142 | 0 |
| 24" | loaded  | 2574 | 1845 (71.7%) | 487 | 100 | 0 | 142 | 0 |
| 30" | bare    | 2574 | 1890 (73.4%) | 380 | 100 | 0 | 204 | 0 |
| 30" | loaded  | 2574 | 1867 (72.5%) | 380 | 100 | 0 | 204 | 23 |
| 36" | bare    | 2574 | 1880 (73.0%) | 353 | 74  | 4  | 263 | 0 |
| 36" | loaded  | 2574 | 1694 (65.8%) | 353 | 74  | 4  | 263 | 186 |

Loading doesn't change geometry rejections (chassis, self-collision, floor) because the geometry is the same. It only changes tipping, because the 3 lb payload shifts the CG further outboard.

---

## Raw Torque Data (All 2574 Poses, Unfiltered)

For reference, these are the torques before any filtering. Includes poses where the arm passes through the chassis, through itself, or underground.

| Config | J2 Peak (Nm) | J3 Peak (Nm) | J4 Peak (Nm) | J2 Avg (Nm) | J3 Avg (Nm) | J4 Avg (Nm) |
|--------|:------------:|:------------:|:------------:|:-----------:|:-----------:|:-----------:|
| 24" bare   |  8.50 |  3.08 | 0.25 | 3.78 | 1.81 | 0.16 |
| 24" loaded | 16.56 |  7.09 | 0.93 | 6.86 | 3.95 | 0.59 |
| 30" bare   | 10.58 |  3.78 | 0.38 | 4.73 | 2.18 | 0.24 |
| 30" loaded | 20.65 |  8.80 | 1.40 | 8.53 | 4.75 | 0.89 |
| 36" bare   | 12.53 |  4.77 | 0.50 | 5.45 | 2.74 | 0.32 |
| 36" loaded | 24.60 | 11.09 | 1.88 | 9.89 | 5.96 | 1.18 |

---

## Flat Environment Data (Supplementary)

The sweep was also run on a flat surface (no step). **This data is not reliable for motor sizing.** On the flat surface, the chassis is held in place only by wheel friction. When the arm teleports to each pose, the sudden CG shift causes the chassis to slide, and the measured joint torques include dynamic forces from the moving base -- not just gravity. Physics-vs-analytical torque agreement on flat is off by 100-200%, compared to <0.5% on the step.

The step environment mechanically constrains the chassis via the step edge, so the arm settles to pure gravity holding and the measurements are accurate.

Flat data is included here only for completeness. **Use step-environment numbers for all design decisions.**

| Config | J2 Peak (Nm) | J3 Peak (Nm) | J4 Peak (Nm) | J2 Avg (Nm) | J3 Avg (Nm) | J4 Avg (Nm) |
|--------|:------------:|:------------:|:------------:|:-----------:|:-----------:|:-----------:|
| 24" bare   | 17.37 | 10.55 | 0.75 | 3.67 | 1.93 | 0.16 |
| 24" loaded | 22.80 | 11.63 | 1.67 | 6.71 | 4.15 | 0.59 |
| 30" bare   | 13.17 |  5.86 | 0.71 | 4.52 | 2.27 | 0.24 |
| 30" loaded | 33.73 | 16.95 | 2.74 | 8.53 | 5.00 | 0.89 |
| 36" bare   | 25.39 | 11.74 | 1.19 | 5.74 | 3.05 | 0.34 |
| 36" loaded | 37.08 | 24.25 | 4.21 | 10.01 | 6.32 | 1.20 |

---

## Data Files

Each configuration produces four files:

| File | Contents |
|------|----------|
| `{config}.csv` | Raw sweep data (2574 rows) |
| `{config}_filtered.csv` | Filtered data with rejection reasons and stability margins |
| `{config}_summary.json` | Sweep metadata (grid, timing, convergence stats) |
| `{config}_filter_summary.json` | Filter statistics and tipping summary |

Config naming: `{24,30,36}in_{bare,loaded}_{step,flat}`
