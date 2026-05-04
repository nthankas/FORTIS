# Deprecated

Files here are superseded and kept only for reference. Don't import from
them, don't copy from them, don't run them expecting reproducible
results.

`scripts/` is split into `arm/` and `chassis/` so you can find the
deprecated counterpart of an active script next to the family it
belonged to (an arm sweep with `arm/`, a reactor-with-arm script with
`chassis/`).

## scripts/arm/

Old arm scripts and IK modules. The active arm pipeline is `v3` --
canonical script `canonical/xdrive_reactor_arm_v3.py`, IK
`lib/arm_ik_v3.py`, sweep tools `tools/arm_continuous_sweep_v3.py` /
`tools/arm_sweep_filter_v3.py` / `tools/arm_sweep_plot_v3.py`.

| Script | What it was | Why deprecated |
|---|---|---|
| `arm_ik_v1.py` | v1 IK / mass / gravity-torque module (square 0.79"x0.79" CF tubes, all-NEMA-17 + Cricket joints) | Superseded by `lib/arm_ik_v2.py` (rectangular tubes, heterogeneous motors), then by `lib/arm_ik_v3.py` (current 1.128"x1.128" CF, NEMA 23 + EG23 J2, always 30" + 3 lb loaded). |
| `arm_continuous_sweep_v1.py` | v1 raw torque sweep (imports `arm_ik_v1`) | Superseded by `tools/arm_continuous_sweep_v2.py` and `tools/arm_continuous_sweep_v3.py`. |
| `arm_sweep_filter_v1.py` | v1 sweep post-filter (imports `arm_ik_v1`) | Superseded by `tools/arm_sweep_filter_v2.py` and `tools/arm_sweep_filter_v3.py`. |
| `arm_stability_sweep_v1.py` | Physics tipping sweep across 24/30/36" arms (imports `arm_ik_v1`) | Tied to v1 hardware/IK. Only the 30" CF v3 build matters now; would need re-implementation against `arm_ik_v3` if revived. |
| `arm_pose_sweep.py` | Isaac Sim sweep with collision ON, continuous motion | Cascading failures: NaN crashes, J1 yaw spin, false tipping. Only 1/1643 poses settled. Replaced by collision-free teleport sweeps. |
| `arm_torque_sweep.py` | Analytical-only torque sweep (no sim) | Superseded by `arm_continuous_sweep_v*.py` which validates analytical torques against physics. |
| `arm_workspace_sweep.py` | IK-based Cartesian workspace sweep | Superseded by joint-space sweep + post-filter, which covers the full workspace more directly. |
| `parse_sweep_results.py` | Parser for old sweep CSV format | Old CSV format no longer produced. |
| `arm_sweep_masses.md` | Mass-calculation notes | Values now live in `lib/arm_ik_v2.py` and `lib/arm_ik_v3.py`. |

## scripts/chassis/

Old chassis-and-reactor scripts plus chassis-with-arm builds. The active
chassis is `canonical/xdrive_realwheel.py`; the active chassis-with-arm
is `canonical/xdrive_reactor_arm_v3.py`.

| Script | What it was | Why deprecated |
|---|---|---|
| `xdrive_reactor_arm.py` | Earliest reactor-with-arm sim | Superseded by the v1/v2/v3 progression below. |
| `xdrive_reactor_arm_v1.py` | v1 arm canonical (all NEMA-17 + Cricket, 0.79"x0.79" CF, configurable 24"/30"/36" + `--metal`/`--cf` link material) | J2 saturated at 19.3 Nm vs 12 Nm Cricket rating. Replaced by v2. |
| `xdrive_reactor_arm_v2.py` | v2 arm canonical (NEMA 23 + EG23 J2, 1.25"x1.38" rectangular CF, camera at L4 midpoint) | Superseded by v3 once the J2 stack settled on NEMA 23 + EG23-G20-D10 20:1 + adapter and the link tube changed to 1.128"x1.128" square CF. v2 is kept so v3 sweep results can be back-compared. |
| `xdrive_reactor.py` | Original reactor sim (no arm) | Superseded by `canonical/xdrive_realwheel.py` with `--reactor`. |
| `xdrive_reactor_v2.py` | Reactor sim v2 with belly rollers | Superseded by canonical realwheel + `--reactor`. |
| `xdrive_r0_entry_v2.py` | R0 port entry sim (early prototype) | Sim 2 (R0 entry) hasn't been started against the canonical script yet; plan lives in `docs/R0_ENTRY_PLAN.md`. |
| `xdrive_o3dyn.py` | Early O3DYN-based sim | Replaced by the custom omni-wheel realwheel build. |
| `xdrive_realwheel_axial.py` | Realwheel variant with axial roller arrangement | Rejected experiment; current build uses radial rollers per `omniwheels.usd`. |
| `xdrive_realwheel_pid.py` | Realwheel variant with PID wheel control | Replaced by velocity-control path in `canonical/xdrive_realwheel.py`. |
| `xdrive_realwheel_7sphere.py` | Realwheel roller collider with 7-sphere approximation | Replaced by the 5-sphere collider in `canonical/xdrive_realwheel.py` -- 5 sphere matched single-sphere baseline torques while preventing the wheel-sink failure mode. |
| `xdrive_realwheel_v2_REJECTED.py` | Realwheel v2 prototype | Self-explanatory; never made canonical. |
| `orbit_torque.py` | Drive-torque profiling at orbit radii on the legacy chassis dims | Superseded by `tools/sweep_orbit_realwheel.py` (validates the canonical realwheel chassis with the 5-sphere collider). |
| `orbit_torque_v2.py` | Orbit-torque profiling on the rectangular-skeleton chassis | Same -- superseded by `sweep_orbit_realwheel.py`, which runs on the canonical realwheel directly. |
| `step_arch_optimizer.py` | Auto-tuning belly-arch geometry against step-clearance | Belly geometry is now locked in `canonical/xdrive_realwheel.py`; tool kept for archival. |
| `sweep_orbit_variant.py` | Earlier orbit-sweep tool, variant-based | Replaced by `sweep_orbit_realwheel.py`. |

## results/

Old result directories from the deprecated scripts. Data is unreliable
or tied to obsolete hardware -- do not use these for sizing or
hardware decisions.

| Directory | What it was | Why deprecated |
|---|---|---|
| `arm_pose_sweep/` | Results from collision-ON continuous sweep | Only 1 pose settled out of 1643. Data is garbage. |
| `arm_pose_sweep_ik/` | Results from IK-based pose sweep | Drive-gain issues caused 99.7% timeouts. |
| `arm_torque_sweep/` | Analytical-only torque sweep plots | Superseded by physics-validated sweeps. |
| `arm_workspace_sweep/` | IK workspace sweep CSVs | Superseded by joint-space collision-free sweep. |
| `arm_continuous_sweep_v1/` | v1 arm sweep CSVs (24/30/36" x bare/loaded x flat/step) + `sweep_report.md`/`.pdf` + `generate_pdf.py` | v1 arm hardware obsolete. |
| `arm_stability_sweep_v1/` | v1 stability sweep tipping data | v1-only tooling; would need re-implementation against v3 IK. |

## Current approach (for reference)

For the active arm, see `tools/arm_continuous_sweep_v3.py` (collision-free physics sweep) + `tools/arm_sweep_filter_v3.py` (analytical post-filter) + `tools/arm_sweep_plot_v3.py` (plots and tables). Results in `results/arm_continuous_sweep/` -- v3 outputs use the `_v3` suffix; v2 outputs are kept alongside for back-comparison; v1 CSVs are in `deprecated/results/arm_continuous_sweep_v1/`.

For the active chassis, see `canonical/xdrive_realwheel.py` and `tools/sweep_orbit_realwheel.py`. Results in `results/orbit_realwheel_5sphere/` and `results/orbit_realwheel_singlesphere/` (the latter kept as the baseline that the 5-sphere collider was validated against).
