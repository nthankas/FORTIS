# fortis_comms

X-drive kinematics and shared QoS profiles, packaged as an ament_python ROS 2 library. Consumed by `fortis_drive`, `fortis_safety`, `fortis_localization`, and `fortis_arm`. Motor-control integration lives in `fortis_control` (via `odrive_ros2_control`); state estimation lives in `fortis_localization` (`robot_localization` EKF).

## Contents

| Module | Purpose |
|---|---|
| `xdrive_kinematics` | X-drive forward / inverse kinematics (4 omni wheels at 45 deg). Module-level functions, no class wrapper. |
| `qos_profiles` | `latched_qos_profile()` — the FORTIS "latched" QoS (TRANSIENT_LOCAL + RELIABLE, depth 1) used by every state-on-connect topic (`/fortis/mission_state`, `/fortis/drive/armed`, `/fortis/context/*`, ...). Single source of truth: a QoS mismatch makes DDS silently refuse the match, so publishers and subscribers must never hand-roll this profile. |

## Tests

| File | Purpose |
|---|---|
| `test/test_imports.py` | Smoke test: the public modules import, IK/FK on zero input return zero, and `latched_qos_profile` stays depth=1 / TRANSIENT_LOCAL / RELIABLE. |
| `test/test_xdrive_kinematics.py` | IK/FK round-trip across forward, strafe, rotation, diagonal, and combined commands. |
| `test/test_kinematics_urdf_sync.py` | Drift regression: parses `fortis_description/urdf/fortis_constants.xacro` and asserts `LEN_X`, `LEN_Y`, `WHEEL_RADIUS` track the URDF (1e-4 m tolerance), plus four-way wheel symmetry. |

Lint (flake8, pep257) is no longer part of `colcon test`. It runs via pre-commit hooks and the `pre-commit` job in `.github/workflows/ci.yml`. See the root README "Pre-commit hooks" section.

Run from the workspace root inside the dev container:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fortis_comms
source install/setup.bash
colcon test --packages-select fortis_comms
colcon test-result --verbose
```

## Drift invariants

The X-drive IK is coupled to chassis geometry, and the URDF in `fortis_description` is the canonical source for that geometry. Three constants in `xdrive_kinematics` must stay in lockstep with the URDF:

| Constant | URDF property | Lives in |
|---|---|---|
| `LEN_X` | `wheel_x_offset` | `fortis_description/urdf/fortis_constants.xacro` |
| `LEN_Y` | `wheel_y_offset` | `fortis_description/urdf/fortis_constants.xacro` |
| `WHEEL_RADIUS` | `wheel_radius`   | `fortis_description/urdf/fortis_constants.xacro` |

`test/test_kinematics_urdf_sync.py` parses the xacro as plain XML and asserts each constant matches the URDF to within 1e-4 m. It also confirms the four `omni_wheel` invocations in `fortis_chassis.urdf.xacro` place the wheels at all four sign combinations of (+/- `wheel_x_offset`, +/- `wheel_y_offset`). If either side drifts, this test fails.

### Known open question

The H-matrix in `xdrive_kinematics.py` uses (`LEN_X` + `LEN_Y`) as a single effective lever arm for the omega column; the per-wheel signs on the Vy and omega columns are preserved verbatim from the senior-design module and have NOT been re-derived from first principles against the URDF wheel yaws (FL=-45 deg, FR=+45 deg, RL=+45 deg, RR=-45 deg). Round-trip IK/FK tests cannot catch a sign-only error because they cancel through the pseudoinverse. The drivetrain has since been bench-driven with the boundary sign corrections in `fortis_drive` (`WHEEL_DIRECTION`, `CMD_VEL_FRAME_SIGN`), so the combined pipeline is verified — but the H-matrix itself remains un-derived. Keep the matrix frozen; correct signs at the hardware boundary, as `drive_node` does.
