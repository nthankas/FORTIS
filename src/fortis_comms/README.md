# fortis_comms

X-drive kinematics, packaged as an ament_python ROS 2 library. Originally a pre-ROS Python module under `control/fortis_comms`; promoted to a first-class colcon package so consumers (`fortis_drive` today) can declare it as a dependency instead of importing through a `sys.path` shim.

The package previously also shipped `motor_base.py`, `odrive_s1.py`, and `ekf.py` interim helpers. Those have been retired to `legacy/deprecated_motor_stack/` and `legacy/deprecated_ekf/` and will be replaced by upstream packages (`odrive_ros2_control`, `robot_localization`). See `docs/CLEANUP_AUDIT.md` Section 2 for the migration plan.

## Contents

| Module | Purpose |
|---|---|
| `xdrive_kinematics` | X-drive forward / inverse kinematics (4 omni wheels at 45 deg). Module-level functions, no class wrapper. |

## Tests

| File | Purpose |
|---|---|
| `test/test_imports.py` | Smoke test: the public module imports and IK/FK on zero input return zero. |
| `test/test_xdrive_kinematics.py` | IK/FK round-trip across forward, strafe, rotation, diagonal, and combined commands. |
| `test/test_kinematics_urdf_sync.py` | Drift regression: parses `fortis_description/urdf/fortis_constants.xacro` and asserts `LEN_X`, `LEN_Y`, `WHEEL_RADIUS` track the URDF (1e-4 m tolerance), plus four-way wheel symmetry. |
| `test/test_ekf.py` | Empty no-op module. The EKF moved to `legacy/deprecated_ekf/ekf.py`; the file is kept as a docstring breadcrumb pointing at the new location. |

Lint (flake8, pep257) is no longer part of `colcon test`. It runs via pre-commit hooks and the `pre-commit` job in `.github/workflows/ci.yml`. See the root README "Pre-commit hooks" section.

Run from the workspace root inside the dev container:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fortis_comms
source install/setup.bash
colcon test --packages-select fortis_comms
colcon test-result --verbose
```

## Migration from `control/fortis_comms`

The legacy location at `control/fortis_comms` has been deleted. The X-drive kinematics moved here unchanged. The accompanying motor/EKF helpers were retired to `legacy/deprecated_motor_stack/` and `legacy/deprecated_ekf/` (replaced by `ros2_control` + standard ROS 2 packages); their historical bug notes and constructor evolution live in those `legacy/` files now.

## Drift invariants

The X-drive IK is coupled to chassis geometry, and the URDF in `fortis_description` is the canonical source for that geometry. Three constants in `xdrive_kinematics` must stay in lockstep with the URDF:

| Constant | URDF property | Lives in |
|---|---|---|
| `LEN_X` | `wheel_x_offset` | `fortis_description/urdf/fortis_constants.xacro` |
| `LEN_Y` | `wheel_y_offset` | `fortis_description/urdf/fortis_constants.xacro` |
| `WHEEL_RADIUS` | `wheel_radius`   | `fortis_description/urdf/fortis_constants.xacro` |

`test/test_kinematics_urdf_sync.py` parses the xacro as plain XML and asserts each constant matches the URDF to within 1e-4 m. It also confirms the four `omni_wheel` invocations in `fortis_chassis.urdf.xacro` place the wheels at all four sign combinations of (+/- `wheel_x_offset`, +/- `wheel_y_offset`). If either side drifts, this test fails.

### Known open question

The H-matrix in `xdrive_kinematics.py` uses (`LEN_X` + `LEN_Y`) as a single effective lever arm for the omega column; the per-wheel signs on the Vy and omega columns are preserved verbatim from the senior-design module and have NOT been re-derived from first principles against the URDF wheel yaws (FL=-45 deg, FR=+45 deg, RL=+45 deg, RR=-45 deg). Round-trip IK/FK tests cannot catch a sign-only error because they cancel through the pseudoinverse. Resolving this is a separate task; it should land before the first real-robot bring-up.
