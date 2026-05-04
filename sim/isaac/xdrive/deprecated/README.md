# Deprecated

Files here are superseded and kept only for reference.

## scripts/

Old simulation and analysis scripts replaced by the collision-free sweep approach.

| Script | What it was | Why deprecated |
|--------|-------------|----------------|
| `arm_pose_sweep.py` | Isaac Sim sweep with collision ON, continuous motion | Cascading failures: NaN crashes, J1 yaw spin, false tipping. Only 1/1643 poses settled. |
| `arm_torque_sweep.py` | Analytical-only torque sweep (no sim) | Superseded by `arm_continuous_sweep.py` which validates analytical torques against physics. |
| `arm_workspace_sweep.py` | IK-based Cartesian workspace sweep | Superseded by joint-space sweep + post-filter which covers the full workspace more directly. |
| `parse_sweep_results.py` | Parser for old sweep CSV format | Old CSV format no longer produced. |
| `arm_sweep_masses.md` | Mass calculation notes | Values now in `lib/arm_ik.py`. |
| `xdrive_reactor_arm.py` | Early reactor sim with arm | Superseded by `canonical/xdrive_reactor_arm.py`. |
| `xdrive_reactor_v2.py` | Reactor sim v2 with belly rollers | Superseded by canonical version. |
| `xdrive_reactor.py` | Original reactor sim | Superseded by canonical version. |
| `xdrive_r0_entry_v2.py` | R0 port entry sim | Superseded by canonical version. |
| `xdrive_o3dyn.py` | Early O3DYN-based sim | Replaced by custom omni-wheel build. |
| `xdrive_reactor_arm_v1.py` | v1 arm canonical (all NEMA-17 + Cricket, 0.79"x0.79" CF) | J2 saturated at 19.3 Nm vs 12 Nm Cricket rating. Replaced by `canonical/xdrive_reactor_arm_v2.py` (NEMA 23 at J2, 1.25"x1.38" CF, 30" CF is the build target). |
| `arm_ik_v1.py` | v1 IK / mass / gravity-torque module | Superseded by `lib/arm_ik_v2.py` (rectangular tube cross-section, v2 mass table, camera at L4 midpoint). |
| `arm_continuous_sweep_v1.py` | v1 raw torque sweep (imports `arm_ik_v1`) | Superseded by `tools/arm_continuous_sweep_v2.py`. |
| `arm_sweep_filter_v1.py` | v1 sweep post-filter (imports `arm_ik_v1`) | Superseded by `tools/arm_sweep_filter_v2.py`. |
| `arm_stability_sweep_v1.py` | Physics tipping sweep across 24/30/36" arms (imports `arm_ik_v1`) | Tied to v1 hardware/IK. Only the 30" CF v2 build matters now; would need re-implementation against `arm_ik_v2` if revived. |

## results/

Old sweep results from deprecated scripts. Data is unreliable.

| Directory | What it was | Why deprecated |
|-----------|-------------|----------------|
| `arm_pose_sweep/` | Results from collision-ON continuous sweep | Only 1 pose settled out of 1643. Data is garbage. |
| `arm_pose_sweep_ik/` | Results from IK-based pose sweep | Drive gain issues caused 99.7% timeouts. |
| `arm_torque_sweep/` | Analytical-only torque sweep plots | Superseded by physics-validated sweep. |
| `arm_workspace_sweep/` | IK workspace sweep CSVs | Superseded by joint-space collision-free sweep. |
| `arm_continuous_sweep_v1/` | v1 arm sweep CSVs across 24/30/36" x bare/loaded x flat/step + `sweep_report.md`/`.pdf` + `generate_pdf.py` | v1 arm hardware obsolete; only 30" CF v2 is the active build target. v2 outputs remain in `results/arm_continuous_sweep/`. |
| `arm_stability_sweep_v1/` | v1 stability sweep tipping data (`36in_loaded_step_tilt.json`, `stability_analysis.md`) | v1-only tooling; would need re-implementation against v2 IK. |

## Current approach

See `tools/arm_continuous_sweep_v2.py` (collision-free physics sweep) + `tools/arm_sweep_filter_v2.py` (analytical post-filter). Results in `results/arm_continuous_sweep/` (v2 files use the `_v2` suffix; legacy v1 CSVs from the same directory are kept for back-comparison but should not be used for sizing).
