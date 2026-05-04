# Step Stability Analysis

## Key Finding

**The robot cannot self-stabilize on the 4.5" step.** The omni wheel lateral spacing (184mm) combined with the 4.5" (114mm) step height creates a ~32 deg chassis tilt. At this angle, the chassis center of gravity has only **10.7mm** of margin before falling off the support polygon -- before any arm loading.

In the Isaac Sim physics test, the robot slides off the step within seconds. The omni wheel rollers provide zero lateral friction, so nothing prevents the chassis from drifting to one floor level. **The tether cable is mandatory for step straddling.**

## Step Tilt Geometry

| Parameter | Value |
|-----------|-------|
| Step height | 4.5" (114.3 mm) |
| Wheel lateral span | 184 mm (sim geometry) |
| Geometric tilt angle | 31.9 deg |
| Chassis CG height | 154 mm above wheel contacts |
| CG lateral shift at tilt | 81.2 mm |
| Support polygon half-width | 91.8 mm |
| **Bare chassis margin** | **10.7 mm** |

The bare chassis alone is barely stable (10.7mm margin). Any arm extension reduces this further.

## Tilt-Corrected Tipping Analysis

Comparison of level-chassis analytical tipping (old) vs tilt-corrected (new), for geometry-valid poses only:

| Config | Level: Stable | Tilted: Stable | New Tipping Poses | Tilted Worst Margin |
|--------|:------------:|:--------------:|:-----------------:|:-------------------:|
| 24" bare   | 1845 / 1845 | 1845 / 1845 |   +0  | 28.1 mm |
| 24" loaded | 1845 / 1845 | 1536 / 1845 | +309  | -13.3 mm |
| 30" bare   | 1890 / 1890 | 1890 / 1890 |   +0  | 17.1 mm |
| 30" loaded | 1867 / 1890 | 1171 / 1890 | +696  | -33.4 mm |
| 36" bare   | 1880 / 1880 | 1880 / 1880 |   +0  | 7.0 mm |
| 36" loaded | 1694 / 1880 |  895 / 1880 | +799  | -52.9 mm |

**All bare configs remain stable** even with the step tilt. The arms are light enough that the CG shift from the arm doesn't exceed the remaining 10.7mm margin.

**Loaded configs are significantly impacted.** The 3 lb payload at the end effector shifts the CG enough to cause tipping in many extended poses:
- 24" loaded: 309 poses (17%) now tip
- 30" loaded: 719 poses (38%) now tip
- 36" loaded: 985 poses (52%) now tip -- over half the workspace is unstable

## What This Means

1. **Bare arm operation on the step is feasible** for all three arm lengths. Margins are reduced but positive.

2. **Loaded arm operation requires cable tension.** Without the tether providing lateral stabilization force, over half the 36" loaded workspace causes tipping.

3. **The tether cable is not optional** -- it is a structural stability requirement, not just a safety tether. The cable must provide at least enough lateral force to counteract the tipping moment from the arm payload.

4. **Motor sizing is unaffected.** The torque data from the collision-free sweep is valid regardless of tipping -- the arm joints see the same gravity loads. Tipping is a chassis-level problem, not a joint-level problem.

## Follow-Up Needed

- Quantify the cable tension required to stabilize each loaded config
- Determine if the cable attachment point can provide adequate restoring moment
- Consider wider wheelbase or different wheel arrangement for better step stability
- Physics validation of borderline poses with constrained chassis (modeling cable)

## Method

The tilt angle is computed from the step geometry and wheel positions. The CG projection is corrected for the tilted gravity vector: each body's apparent position shifts by `height * tan(tilt)` toward the lower side. J1 is swept 0-345 deg in 15 deg steps. A pose is rejected if the CG exits the wheel support polygon at any J1 angle.

The tipping functions are in `lib/arm_ik.py` (`tipping_at_j1_tilted`, `tipping_sweep_j1_tilted`). The physics sim (`tools/arm_stability_sweep.py` with `--measure-tilt`) confirmed the robot slides off the step without lateral constraint.
