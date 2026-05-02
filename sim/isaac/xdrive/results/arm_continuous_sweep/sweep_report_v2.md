# Arm Torque Sweep Report — v2 Hardware (30″ Loaded, Step)

Physics-validated gravity torque sweep of the FORTIS 4-DOF arm with the
new heterogeneous-motor hardware spec. Robot is on the 4.5″ step
environment (deployment scenario). Only the **30″ loaded** configuration
is included; this is the build we are sizing motors against.

The v1 sweep (sibling file `sweep_report.md` in this directory) was the
original all-NEMA-17 + Cricket 25:1 survey. v1 showed that J2 needed
~17.5 Nm continuous to handle a 3 lb payload, against a 12 Nm Cricket
rating — J2 was saturated. v2 swaps in a NEMA 23 + EG23-G20-D10 20:1 at
J2 (30 Nm continuous / 60 Nm peak) and keeps the lighter actuators
everywhere else. This report is the torque validation of that choice.

---

## Hardware (v2)

Per-joint hardware for the 30″ loaded sweep:

| Joint | Motor + gearbox | Mass | Continuous rating | Peak rating |
|---|---|---:|---:|---:|
| J1 (Z yaw) | NEMA 17 (17HS24-2104-ME1K) + Cricket Drive MK II 25:1 | 0.580 kg | 12.0 Nm | — |
| J2 (shoulder) | NEMA 23 (23HS45-4204-ME1K) + EG23-G20-D10 20:1 + adapter | 2.665 kg | 30.0 Nm | 60.0 Nm |
| J3 (elbow) | NEMA 17 (17HS24-2104-ME1K) + Cricket Drive MK II 25:1 + adapter | 0.655 kg | 12.0 Nm | — |
| J4 (wrist) | Hitec D845WP servo (IP67) + adapter | 0.302 kg | 4.9 Nm @ 7.4 V | — |

Links (RockWest Composites 1.25″ × 1.38″ OD CF square tube, 0.065″ wall,
0.226 lb/ft):

| Link | Length | Mass |
|---|---|---:|
| L2 | 15 in | 128 g |
| L3 | 12 in | 103 g |
| L4 |  3 in |  26 g |

End effector (at L4 tip):

| Component | Mass |
|---|---:|
| ServoCity parallel gripper kit | 81 g |
| Hitec D645MW gripper servo | 60 g |
| Adapter | 75 g |
| **Gripper subtotal** | **216 g** |
| Payload (3 lb) | 1361 g |
| **Gripper + payload (loaded)** | **1577 g** |

Instrumentation: Orbbec Gemini 2 depth camera (98 g) is mounted at the
**midpoint of L4**, 1.5″ from the J4 pivot along the link.

Joint ranges actually swept (hardware limits, from the spec):

| Joint | Range |
|---|---:|
| J1 | ±180° (software) |
| J2 | −30° to +150° (physical) |
| J3 | ±170° (self-collision limited) |
| J4 | −100° to +100° (D845WP ±101° hardware limit) |

Total loaded arm mass (sum of components above, excluding chassis): **6.133 kg (13.52 lb)**.

---

## Method

Identical pipeline to the v1 sweep, re-parameterised with v2 masses and
the rectangular tube cross-section. Three stages:

1. **Raw torque sweep** — `tools/arm_continuous_sweep_v2.py` headless in
   Isaac Sim. Arm link self-collision disabled so every joint combo is
   reachable. For each pose the arm is teleported to the target, a PD
   controller holds position for ~0.42 s, then joint torques are
   measured over the next ~0.10 s and averaged. Grid:
   J2 ∈ [−30, 150] / 15°, J3 ∈ [−170, 170] / 20°, J4 ∈ [−100, 100] / 30°
   → **1638 poses**, J1 fixed at 0°.
2. **Post-filter** — `tools/arm_sweep_filter_v2.py` applies analytical
   self-collision (L4 vs L2, L4 vs J1), chassis collision (link
   segments vs chassis AABB), and floor collision checks to the raw
   CSV. Output: a filtered CSV with only geometrically realizable
   poses. v2 uses `arm_ik_v2` with the larger 1.25×1.38 tube, so the
   collision envelope is ~58 % fatter than v1's 0.79″ square.
3. **Analytical cross-check** — for every pose, `arm_ik_v2.gravity_torques`
   computes the static gravity torque independently from the physics
   measurement. The two should agree up to PD steady-state error.

**All 1638 poses converged, 0 NaN.** Position error mean 0.20°, max 0.58°,
p95 0.47°.

### Analytical cross-check (filtered poses only)

| Joint | max \||physics\| − \|analytical\|| | as % of peak physics torque |
|---|---:|---:|
| J2 | 189 mNm | 1.11 % |
| J3 |  73 mNm | 1.12 % |
| J4 |  14 mNm | 1.13 % |

The two methods agree within ~1 % at peak — the residual is the PD
tracking error (arm lands 0.2–0.6° off target, which moves the CG by a
fraction of a degree and perturbs the torque). This is independent
validation of both the Isaac Sim build and the analytical model.

---

## Results — filtered valid poses (n = 1203 / 1638 = 73.4 %)

| Joint | Peak \|τ\| | Avg \|τ\| | p95 \|τ\| | Continuous rating | Utilization (peak / cont.) | Continuous headroom |
|---|---:|---:|---:|---:|---:|---:|
| **J1** (yaw)      |  0.091 Nm | 0.002 Nm | 0.014 Nm | 12.0 Nm |  **0.8 %** | 99.2 % |
| **J2** (shoulder) | 17.059 Nm | 7.061 Nm | 14.906 Nm | 30.0 Nm | **56.9 %** | 43.1 % |
| **J3** (elbow)    |  6.506 Nm | 3.673 Nm |  6.011 Nm | 12.0 Nm | **54.2 %** | 45.8 % |
| **J4** (wrist)    |  1.227 Nm | 0.775 Nm |  1.221 Nm |  4.9 Nm | **25.0 %** | 75.0 % |

Peak poses (worst-case gravity torque in the valid set):

| Joint | Peak pose (J2 / J3 / J4, deg) | Interpretation |
|---|---|---|
| J1 |  120 / −170 / 20 | Residual only; see note below |
| J2 |    0 /  170 / −100 | L2 horizontal forward; L3 flipped forward (3R arm nearly straight out); L4 bent down |
| J3 |  135 / −150 / −100 | Arm rotated up/back and folded — worst-case moment arm on L3 |
| J4 |   30 / −170 / −40 | Gripper load applied off-axis on L4 |

### On J1

J1 rotates about the chassis Z axis. Gravity is also along Z, so the
static gravity torque at J1 is identically **zero** — the moment arm of
every body projected onto a vertical axis collapses. The 0.091 Nm peak
we report is PhysX residual from the teleport/settle dynamics, not a
real load. J1 torque in operation is dominated by **inertial slewing**
(accelerating the arm's moment of inertia about Z), which this sweep
does not measure. If you need a J1 slew torque, compute it from the
rotational inertia of the worst-case pose × angular acceleration — it's
not in this report.

---

## Raw all-pose statistics (n = 1638)

For reference, including poses rejected by the geometry filter:

| Joint | Peak \|τ\| | Avg \|τ\| | p95 \|τ\| | Utilization |
|---|---:|---:|---:|---:|
| J1 |  0.091 Nm | 0.002 Nm |  0.010 Nm |   0.8 % |
| J2 | 17.059 Nm | 7.365 Nm | 14.930 Nm |  56.9 % |
| J3 |  6.506 Nm | 3.550 Nm |  5.983 Nm |  54.2 % |
| J4 |  1.227 Nm | 0.779 Nm |  1.221 Nm |  25.0 % |

Filter rejection reasons (435 poses rejected):

| Reason | Count | % |
|---|---:|---:|
| Chassis collision | 236 | 14.4 % |
| Floor collision | 128 |  7.8 % |
| Self-collision L4 vs L2 |  71 |  4.3 % |
| Self-collision L4 vs J1 |   0 |  0.0 % |

---

## v1 vs v2 comparison (30″ loaded, step)

| Joint | v1 peak | v1 motor rating | v1 utilization | v2 peak | v2 motor rating | v2 utilization |
|---|---:|---:|---:|---:|---:|---:|
| J2 | **19.30 Nm** | 12.0 Nm | **161 %** (saturated) | **17.06 Nm** | 30.0 Nm | 56.9 % |
| J3 |   8.80 Nm | 12.0 Nm |   73.3 % |   6.51 Nm | 12.0 Nm | 54.2 % |
| J4 |   1.40 Nm |  4.9 Nm |   28.6 % |   1.23 Nm |  4.9 Nm | 25.0 % |

(v1 numbers from `sweep_report.md`, 30″ loaded step row.)

A few non-obvious things:

1. **v2 J2 peak is 2.24 Nm *lower* than v1**, even though the v2 arm is
   heavier (6.13 kg vs 2.75 kg). The v2 gripper is 284 g *lighter*
   (216 g vs 500 g bare), and that 284 g sat at the end of a 30″ moment
   arm in v1 — removing it saves ~2.1 Nm at J2. The heavier CF tube and
   heavier J2/J4 bodies add some back, but the tip-mass reduction wins.
2. **The motor swap is what actually fixes v1, not a torque reduction.**
   The v1 J2 torque requirement was 19.3 Nm against a 12 Nm rating =
   saturated. Even holding the v2 torque constant at 19.3 Nm and
   replacing the motor would have dropped utilization from 161 % to
   64 %. The tip-mass reduction is a bonus.
3. **J3 and J4 drop only slightly.** Same hardware as v1 on those
   joints, same CF tube density increase, same 3 lb payload. J3 dropped
   26 % (8.80 → 6.51) because L3 is no longer carrying the 98 g camera
   as a cantilevered load on L2 near J3 (the v1 camera position on L2
   shoulder added a small moment to J3; moving it to L4 midpoint shifts
   the load onto J4 instead). J4 dropped 12 %.
4. **J1 is unchanged** (0 static torque either way) and remains
   inertial-slew-limited in operation.

---

## Motor sizing conclusions

| Joint | Hardware | Peak required | Continuous margin | Peak margin |
|---|---|---:|---:|---:|
| J1 | NEMA 17 + Cricket 25:1 | ~0 Nm static | ∞ | ∞ |
| J2 | NEMA 23 + EG23-G20-D10 20:1 | 17.1 Nm | **1.76× continuous (30 Nm)** | **3.52× peak (60 Nm)** |
| J3 | NEMA 17 + Cricket 25:1 | 6.5 Nm | **1.85× continuous (12 Nm)** | — |
| J4 | Hitec D845WP (7.4 V) | 1.2 Nm | **4.00× continuous (4.9 Nm)** | — |

**All four joints have ≥ 1.76× continuous torque margin at worst-case
static pose.** J2 has 3.52× peak-torque margin, which is the important
number for dynamic moves (accelerating the arm, resisting load
disturbances).

v2 **is** the hardware to build.

---

## Files referenced by this report

| File | Purpose |
|---|---|
| `30in_loaded_step_v2.csv` | Raw per-pose physics + analytical torques, 1638 rows |
| `30in_loaded_step_v2_summary.json` | Stage-1 summary (grid, counts, elapsed) |
| `30in_loaded_step_v2_filtered.csv` | Post-filtered CSV with rejection reasons and analytical tipping column |
| `30in_loaded_step_v2_filter_summary.json` | Stage-2 summary (rejection tallies) |
| `../../canonical/xdrive_reactor_arm_v2.py` | GUI build for visual verification |
| `../../tools/arm_continuous_sweep_v2.py` | Stage 1 (raw sweep) |
| `../../tools/arm_sweep_filter_v2.py` | Stage 2 (post-filter) |
| `../../lib/arm_ik_v2.py` | Analytical FK / gravity torques / per-joint masses |
