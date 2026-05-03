# fortis_comms

Motor abstractions, X-drive kinematics, and EKF, packaged as an ament_python ROS 2 library. Originally a pre-ROS Python module under `control/fortis_comms`; promoted to a first-class colcon package so consumers (`fortis_drive` today, `fortis_arm` next) can declare it as a dependency instead of importing through a `sys.path` shim.

## Contents

| Module | Purpose |
|---|---|
| `motor_base` | Abstract `Motor` base class (connect/disconnect/command_velocity, status enum, context manager). |
| `odrive_s1` | `ODrive_S1` concrete subclass. Sends ODrive 0.5.x CAN frames over python-can. |
| `xdrive_kinematics` | X-drive forward / inverse kinematics (4 omni wheels at 45 deg). Module-level functions, no class wrapper. |
| `ekf` | 6-state EKF (x, y, theta, vx, vy, omega) with optical-flow and IMU updates. Loads default noise covariances from `cfgs/ekf_params.json` via importlib.resources; pass a path or dict to `EKF(config=...)` to override. |
| `cfgs/ekf_params.json` | Bundled default EKF noise covariances. |

## Tests

| File | Purpose |
|---|---|
| `test/test_imports.py` | Smoke test: every public module imports, IK/FK on zero input return zero, `EKF()` constructs from the bundled default config. |
| `test/test_xdrive_kinematics.py` | IK/FK round-trip across forward, strafe, rotation, diagonal, and combined commands. |
| `test/test_ekf.py` | Init, predict, optical-flow update, IMU update, and full-loop stability. Uses an in-fixture dict config rather than monkey-patching. |
| `test/test_flake8.py`, `test/test_pep257.py` | Lint scaffolds, scoped to this package only via `Path(__file__).resolve().parent.parent`. |

Run from the workspace root inside the dev container:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fortis_comms
source install/setup.bash
colcon test --packages-select fortis_comms
colcon test-result --verbose
```

## Migration from `control/fortis_comms`

The legacy location at `control/fortis_comms` has been deleted. Public APIs were preserved for `motor_base`, `odrive_s1`, and `xdrive_kinematics`. The only material change was to `ekf.py`:

- The constructor now accepts an optional `config: str | Path | Mapping | None = None`. Passing nothing loads the bundled `cfgs/ekf_params.json` via `importlib.resources`. Passing a dict or a path overrides.
- The old `EKF.CONFIG_FILE = "/cfgs/ekf_params.json"` class attribute is gone — it was a root-relative path that never resolved at runtime, and the prior tests only passed because the fixture monkey-patched it.
- Missing required keys raise `KeyError`; a path that does not exist raises `FileNotFoundError`. No more `NameError` on a missing config.

## Known issues deferred

These were flagged in the legacy README and remain. They are real behavioural bugs but out of scope for the packaging move; address them in the next pass over `odrive_s1`:

- `odrive_s1.py` line 17: `can.Bus("can0", interface="virtual")` is a contradiction (channel `can0` paired with the `virtual` interface). Real hardware needs `interface="socketcan", channel="can0"`; tests need `interface="virtual"`. Should take these as constructor args.
- `odrive_s1.py` `_stop_motor` line 37: `struct.pack('<ff', 0)` packs one value into a two-float format; will raise. Should be `(0.0, 0.0)` like the other call sites.
- `odrive_s1.py` `_read_position` / `_read_velocity` block on `bus.recv` until a matching frame arrives. Use a callback-driven receive instead of pulling on demand inside a control loop.

## Chassis dimension note

`xdrive_kinematics.LEN_X = 4.405 in` and `LEN_Y = 6.462 in` were copied verbatim from the senior-design module. They do **not** match the locked 13.082 x 8.54 in skeleton dimensions in use today. Reconciling them is a real calibration decision (which is the source of truth, do we re-tune the EKF, etc.) and is intentionally out of scope for this packaging task. Track this as a follow-up before drive bring-up on real hardware.
