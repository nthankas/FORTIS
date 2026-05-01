# fortis_comms

Pre-ROS Python library for motor control, kinematics, and state estimation. Written before the move to ROS 2. Tests pass in isolation; no ROS wrapping yet.

## Contents

| File | Purpose |
|---|---|
| `motor_base.py` | Abstract `Motor` base class (connect/disconnect/command_velocity, status enum, context manager). |
| `odrive_s1.py` | `ODrive_S1` concrete subclass. Sends ODrive 0.5.x CAN frames over python-can. |
| `xdrive_kinematics.py` | X-drive forward / inverse kinematics (4 omni wheels at 45 deg). |
| `ekf.py` | 6-state EKF (x, y, theta, vx, vy, omega) with optical-flow and IMU updates. |
| `cfgs/ekf_params.json` | EKF noise covariances. |
| `test_xdrive_kinematics.py` | Self-contained sanity test for IK/FK round trip. |
| `ekf_test.py` | pytest suite for the EKF (init, predict, update, full-loop stability). |

## Status

| Component | State | Notes |
|---|---|---|
| `motor_base.Motor` | usable as-is | Clean abstract interface. Stays in shape with whatever motor controller node we end up writing. |
| `ODrive_S1` | scaffolding only | Hard-codes `interface="virtual"` CAN bus, has no real-bus path, no heartbeat handling, no error decoding, no ODrive 0.5.x vs 0.6.x distinction. CAN ID layout is right. |
| `xdrive_kinematics` | usable as-is | Pure numpy, correct H matrix for 45-deg corner wheels. |
| `EKF` | usable algorithmically | `CONFIG_FILE = "/cfgs/ekf_params.json"` is an absolute path that won't resolve at runtime; tests patch it via fixture. Needs to take a path or dict in the constructor. |

## Integration path into ROS 2

The intended target package is `fortis_drive` (not yet created, blocked on URDF / `fortis_description`). When that package goes in, the choice is:

1. **Replace ODrive_S1 with the `ros_odrive` community package.** That node already does heartbeat handling, error reporting, `ros2_control` integration, and supports the wire protocol for ODrive Pro / S1. If it works for our use, drop `odrive_s1.py` and let `ros2_control` own the CAN bus. Most likely outcome.
2. **Keep `motor_base.Motor` as the abstract interface, write a thin ROS wrapper around it.** Useful if `ros_odrive` doesn't fit (e.g. we need a behavior it doesn't expose). Reuses the existing class hierarchy but means we maintain our own CAN code.
3. **Throw the whole library away.** Unlikely - the kinematics and the EKF are independent of motor choice and worth keeping.

`xdrive_kinematics.py` and `ekf.py` should survive any of those paths. Wrap them in a ROS node (`fortis_drive/xdrive_controller`, `fortis_localization/ekf_node`) that subscribes to `/cmd_vel` and joint states, runs the math, and publishes wheel commands and odometry. Replace `robot_localization` for our EKF if we don't trust its tuning, otherwise just use `robot_localization` and drop our `ekf.py`.

Decision deadline: when `fortis_description` lands and `fortis_drive` work starts.

## Known bugs to fix before wrapping

- `odrive_s1.py` line 1: `from motor_base import Motor, MotorStatus` is a top-level import that only works when run from `control/fortis_comms/`. Should be `from .motor_base import ...` once this becomes a real package.
- `odrive_s1.py` line 17: `can.Bus("can0", interface="virtual")` is a contradiction (channel `can0` with `virtual` interface). Real hardware needs `interface="socketcan", channel="can0"`; tests need `interface="virtual"`. Pass these in via the constructor.
- `odrive_s1.py` `_stop_motor` line 37: `struct.pack('<ff', 0)` packs one value into a two-float format - will raise. Should be `(0.0, 0.0)` like the other call sites.
- `odrive_s1.py` `_read_position` / `_read_velocity` block on `bus.recv` until a matching frame arrives - blocking with a 1 s timeout means a read can stall the caller for 1 s and a stuck bus throws TimeoutError mid-control loop. Use the encoder estimate stream + a periodic callback instead of pulling on demand.
- `ekf.py` line 6: `CONFIG_FILE = "/cfgs/ekf_params.json"` is absolute and will not resolve at runtime; the tests only work because the fixture rewrites the class attribute. Constructor should take a config path or a dict.
- `ekf.py` line 11: `_ekf_params` is referenced even when the file doesn't exist (the `if os.path.exists` guard skips the load but later lines still read `_ekf_params`). Will `NameError` on a missing config.

These are not blockers for the reorg, but every one of them is something a future ROS wrapping pass will trip over. Listed here so the next Claude / next session doesn't have to rediscover them.
