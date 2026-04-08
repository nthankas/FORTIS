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

## results/

Old sweep results from deprecated scripts. Data is unreliable.

| Directory | What it was | Why deprecated |
|-----------|-------------|----------------|
| `arm_pose_sweep/` | Results from collision-ON continuous sweep | Only 1 pose settled out of 1643. Data is garbage. |
| `arm_pose_sweep_ik/` | Results from IK-based pose sweep | Drive gain issues caused 99.7% timeouts. |
| `arm_torque_sweep/` | Analytical-only torque sweep plots | Superseded by physics-validated sweep. |
| `arm_workspace_sweep/` | IK workspace sweep CSVs | Superseded by joint-space collision-free sweep. |

## Current approach

See `tools/arm_continuous_sweep.py` (collision-free physics sweep) + `tools/arm_sweep_filter.py` (analytical post-filter). Results in `results/arm_continuous_sweep/`.
