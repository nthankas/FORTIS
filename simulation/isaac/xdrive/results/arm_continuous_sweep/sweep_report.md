# Arm Torque Sweep Results

Collision-free physics sweep of all J2/J3/J4 joint combinations for the FORTIS 4-DOF arm.
Each configuration sweeps 2574 poses (coarse grid: J2 [-30,150]/15, J3 [-170,170]/20, J4 [-150,150]/30) at J1=0.

## Method

### Simulation (raw data)

Isaac Sim headless, 360 Hz TGS solver. Collision is **disabled** on all arm links so every joint combination can be reached regardless of geometry. For each pose the arm is teleported to the target angles, the PD controller settles for 150 frames (~0.42 s), then torques are measured over 36 frames (~0.10 s). The physics-measured torques agree with analytical gravity torques to within 0.5%.

All 12 configurations achieved **2574/2574 poses converged, 0 NaN**. Mean position error < 0.1 deg.

### Filtering (valid data)

Because collision is off during simulation, many recorded poses are physically impossible. A post-processing filter removes them using analytical geometry checks from `lib/arm_ik.py`. A pose is rejected if **any** of the following are true:

| Filter | Heuristic |
|--------|-----------|
| **Chassis collision** | Sample 10 points along each link segment (J2-J3, J3-J4, J4-EE). If any point falls inside the chassis bounding box (expanded by 10 mm margin on each side), reject. The chassis box spans from the arm mount backward along the full chassis length, and from J1 stack height down to the chassis bottom. |
| **Self-collision L4-L2** | Compute minimum distance between the L4 segment (J4-EE) and the L2 segment (J2-J3) by sampling 12 points on each segment. If the minimum distance is less than twice the link half-thickness (0.79 in CF tube / 2 + 5 mm clearance ~ 15 mm), reject. |
| **Self-collision L4-J1** | Sample 8 points along L4. If any point falls inside the J1 base volume (30 mm half-width box, height = J1 stack), reject. |
| **Floor collision** | Check if any joint position or link midpoint (via FK) falls below the floor plane, accounting for J2 height above the lowest floor surface and a margin of 10 mm + link half-thickness. |
| **Tipping** | Compute the combined CG of chassis (20.4 kg) + all arm masses projected onto the ground plane. Sweep J1 from 0 to 345 deg in 15 deg steps and check if the CG projection falls inside the axis-aligned bounding box of the 4 wheel contact points. If the CG falls outside at **any** J1 angle, reject. |

Note on tipping: this is a **static CG projection on a level chassis**. It does not account for the step tilt, dynamic effects, or cable tension. Step tilt shifts the effective CG projection and will reduce stability margins on the downhill side. This needs to be addressed in a follow-up analysis.

### Rejection breakdown

Rejection counts are geometry-only (identical for step and flat environments within the same arm/loading).

| Arm | Loading | Total | Valid | Chassis | Self L4-L2 | Self L4-J1 | Floor | Tipping |
|-----|---------|------:|------:|--------:|-----------:|-----------:|------:|--------:|
| 24" | bare    | 2574  | 1845 (71.7%) | 487 | 100 | 0 | 142 | 0 |
| 24" | loaded  | 2574  | 1845 (71.7%) | 487 | 100 | 0 | 142 | 0 |
| 30" | bare    | 2574  | 1890 (73.4%) | 380 | 100 | 0 | 204 | 0 |
| 30" | loaded  | 2574  | 1867 (72.5%) | 380 | 100 | 0 | 204 | 23 |
| 36" | bare    | 2574  | 1880 (73.0%) | 353 | 74  | 4 | 263 | 0 |
| 36" | loaded  | 2574  | 1694 (65.8%) | 353 | 74  | 4 | 263 | 186 |

Loading does not change geometry rejections (chassis, self, floor) but does change tipping because the 3 lb payload at the EE shifts the CG further outboard.

---

## Raw Torque Table (all 2574 poses, no filtering)

Includes poses where the arm passes through the chassis, through itself, or underground. Flat-environment max torques are inflated by dynamic oscillation artifacts (no step tilt to damp the PD overshoot).

| Config | J2 max (Nm) | J3 max (Nm) | J4 max (Nm) | J2 mean (Nm) | J3 mean (Nm) | J4 mean (Nm) |
|--------|------------:|------------:|------------:|--------------:|--------------:|--------------:|
| 24" bare flat    | 17.37 | 10.55 | 0.75 | 3.86 | 1.86 | 0.16 |
| 24" bare step    |  8.50 |  3.08 | 0.25 | 3.78 | 1.81 | 0.16 |
| 24" loaded flat  | 24.00 | 11.63 | 1.67 | 6.96 | 4.01 | 0.59 |
| 24" loaded step  | 16.56 |  7.09 | 0.93 | 6.86 | 3.95 | 0.59 |
| 30" bare flat    | 13.17 |  5.86 | 0.71 | 4.73 | 2.19 | 0.24 |
| 30" bare step    | 10.58 |  3.78 | 0.38 | 4.73 | 2.18 | 0.24 |
| 30" loaded flat  | 36.89 | 16.95 | 2.95 | 8.70 | 4.84 | 0.90 |
| 30" loaded step  | 20.65 |  8.80 | 1.40 | 8.53 | 4.75 | 0.89 |
| 36" bare flat    | 25.39 | 11.74 | 1.19 | 5.83 | 2.92 | 0.34 |
| 36" bare step    | 12.53 |  4.77 | 0.50 | 5.45 | 2.74 | 0.32 |
| 36" loaded flat  | 39.18 | 24.25 | 4.21 | 10.14 | 6.09 | 1.21 |
| 36" loaded step  | 24.60 | 11.09 | 1.88 |  9.89 | 5.96 | 1.18 |

## Filtered Torque Table (valid poses only)

Only poses that pass all five filters. These are the torques for physically realizable arm configurations. Step-environment values are the deployment-relevant numbers.

| Config | N valid | J2 max (Nm) | J3 max (Nm) | J4 max (Nm) | J2 mean (Nm) | J3 mean (Nm) | J4 mean (Nm) |
|--------|--------:|------------:|------------:|------------:|--------------:|--------------:|--------------:|
| 24" bare flat    | 1845 | 17.37 | 10.55 | 0.75 | 3.67 | 1.93 | 0.16 |
| 24" bare step    | 1845 |  8.50 |  3.08 | 0.25 | 3.54 | 1.88 | 0.16 |
| 24" loaded flat  | 1845 | 22.80 | 11.63 | 1.67 | 6.71 | 4.15 | 0.59 |
| 24" loaded step  | 1845 | 16.56 |  7.09 | 0.93 | 6.59 | 4.11 | 0.59 |
| 30" bare flat    | 1890 | 13.17 |  5.86 | 0.71 | 4.52 | 2.27 | 0.24 |
| 30" bare step    | 1890 | 10.58 |  3.78 | 0.38 | 4.50 | 2.26 | 0.24 |
| 30" loaded flat  | 1867 | 33.73 | 16.95 | 2.74 | 8.53 | 5.00 | 0.89 |
| 30" loaded step  | 1867 | 19.30 |  8.80 | 1.40 | 8.32 | 4.92 | 0.89 |
| 36" bare flat    | 1880 | 25.39 | 11.74 | 1.19 | 5.74 | 3.05 | 0.34 |
| 36" bare step    | 1880 | 12.53 |  4.77 | 0.50 | 5.22 | 2.84 | 0.32 |
| 36" loaded flat  | 1694 | 37.08 | 24.25 | 4.21 | 10.01 | 6.32 | 1.20 |
| 36" loaded step  | 1694 | 19.42 | 11.09 | 1.87 |  8.47 | 5.89 | 1.19 |

---

## Motor Sizing (step environment, filtered)

These are the numbers to design against. Step environment is the real deployment scenario.

| Config | J2 peak | J3 peak | J4 peak |
|--------|--------:|--------:|--------:|
| 24" bare    |  8.5 Nm |  3.1 Nm | 0.3 Nm |
| 24" loaded  | 16.6 Nm |  7.1 Nm | 0.9 Nm |
| 30" bare    | 10.6 Nm |  3.8 Nm | 0.4 Nm |
| 30" loaded  | 19.3 Nm |  8.8 Nm | 1.4 Nm |
| 36" bare    | 12.5 Nm |  4.8 Nm | 0.5 Nm |
| 36" loaded  | 19.4 Nm | 11.1 Nm | 1.9 Nm |

---

## Chassis Stability (analytical CG projection, level chassis)

Static tipping analysis: combined CG of chassis + arm projected onto the ground plane, checked against the wheel support polygon (AABB of 4 wheel contact points). J1 is swept 0-345 deg in 15 deg steps; the worst-case angle is reported.

**This does not account for step tilt.** The 4.5" step creates a permanent chassis tilt that shifts the CG projection. Stability margins on the downhill side will be reduced. Follow-up needed.

| Config | Tip-rejected | Worst margin | Mean margin | Tight (<10 mm) |
|--------|-------------:|-------------:|------------:|---------------:|
| 24" bare    |   0 (0.0%) | 43.0 mm / 1.69" | 74.2 mm | 0 |
| 24" loaded  |   0 (0.0%) | 11.4 mm / 0.45" | 45.7 mm | 0 |
| 30" bare    |   0 (0.0%) | 33.8 mm / 1.33" | 63.0 mm | 0 |
| 30" loaded  |  23 (1.2%) |  0.5 mm / 0.02" | 35.5 mm | 139 (7.4%) |
| 36" bare    |   0 (0.0%) | 25.2 mm / 0.99" | 54.7 mm | 0 |
| 36" loaded  | 186 (9.9%) |  0.0 mm / 0.00" | 28.4 mm | 151 (8.9%) |

The tipping-rejected poses are all fully-extended configurations (J3 near +/-170 deg, J4 near +/-150 deg) with worst-case J1 = 90 deg (arm reaching sideways, perpendicular to the long axis of the chassis). The chassis is narrower in Y (~9.4") than in X (~15.4"), so sideways reach produces the smallest stability margin.

---

## Files

| File | Description |
|------|-------------|
| `{config}.csv` | Raw sweep data (2574 rows), all columns |
| `{config}_filtered.csv` | Filtered data with rejection reasons + stability margins |
| `{config}_summary.json` | Sweep metadata (grid, timing, convergence) |
| `{config}_filter_summary.json` | Filter statistics + tipping summary |

Config names: `{24,30,36}in_{bare,loaded}_{step,flat}`

### CSV columns (raw)

`pose_idx, j2_deg, j3_deg, j4_deg, j2_fk_deg, j3_fk_deg, j4_fk_deg, j1_actual_deg, j2_actual_deg, j3_actual_deg, j4_actual_deg, position_error_deg, tau_j1_Nm, tau_j2_Nm, tau_j3_Nm, tau_j4_Nm, tau_j2_analytical, tau_j3_analytical, tau_j4_analytical, ee_x_m, ee_y_m, ee_z_m, j3_local_x_m, j3_local_z_m, j4_local_x_m, j4_local_z_m, ee_local_x_m, ee_local_z_m, chassis_roll_deg, chassis_pitch_deg, converged`

### Additional columns (filtered)

`filter_status, filter_chassis, filter_self_L4_L2, filter_self_L4_J1, filter_floor, filter_converged, filter_tipping, tip_worst_margin_m, tip_worst_j1_deg, tip_j1_stable_count, tip_cg_x_m, tip_cg_y_m`
