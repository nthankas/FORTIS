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

## Chassis dimension note

`xdrive_kinematics.LEN_X = 4.405 in` and `LEN_Y = 6.462 in` were copied verbatim from the senior-design module. They do **not** match the locked 13.082 x 8.54 in skeleton dimensions in use today. Reconciling them is a real calibration decision (which is the source of truth, do we re-tune the EKF, etc.) and is intentionally out of scope for this packaging task. Track this as a follow-up before drive bring-up on real hardware.
