# fortis_localization

Planar state estimation for the FORTIS X-drive base: wheel odometry plus a
`robot_localization` EKF that fuses it with the front OAK-D Lite IMU into
`/odometry/filtered` and the `odom -> base_link` TF.

## Nodes

| Executable | What it does |
|---|---|
| `wheel_odometry_node` | Reads the four wheel joint velocities from `/joint_states` BY NAME (the broadcaster does not guarantee array order), recovers the body twist via the X-drive FK from `fortis_comms`, dead-reckons a planar pose using header-stamp intervals, and publishes `/odom`. Does NOT broadcast TF — the EKF owns `odom -> base_link`, and two TF publishers would fight. |
| `imu_gyro_debias_node` | Estimates the gyro zero-rate bias whenever the drive is DISARMED (`/fortis/drive/armed` false — the motors are idle, so the chassis is physically still by definition) and subtracts it from every IMU sample, publishing `/imu/debiased`. The estimate freezes while armed and skips a settle window after each disarm edge (the chassis rocks as the motors release). |
| `ekf_filter_node` | `robot_localization`'s stock `ekf_node`, launched from this package with `config/ekf.yaml` (or `config/ekf_vio.yaml` when `vio:=true`). |

## Running

```bash
ros2 launch fortis_localization localization.launch.py
```

Launch arguments:

| Arg | Default | Meaning |
|---|---|---|
| `imu_topic` | `/oak_chassis_front/imu/data` | Source IMU topic, remapped to the EKF's `/imu` (depthai v3 nests the front OAK's IMU under `.../imu/data`). |
| `imu_frame` | `oak_chassis_front_imu_frame` | frame_id the IMU messages carry; used as the child of the static IMU->base_link transform. VERIFY against the live stream: `ros2 topic echo <imu_topic> --field header.frame_id`. |
| `publish_imu_tf` | `true` | Publish an identity static imu_frame->base_link transform so localization works standalone (the FORTIS URDF does not model the camera's internal IMU frame). Set `false` when the camera launch already supplies the full chain, to avoid a duplicate TF. |
| `debias_imu` | `true` | Insert `imu_gyro_debias_node` between the raw IMU and the EKF (EKF reads `/imu/debiased`). `false` = EKF reads `imu_topic` directly, byte-for-byte the pre-debias behaviour. |
| `vio` | `false` | `true` selects `config/ekf_vio.yaml`, which additionally fuses the RGBD visual-odometry body twist `/fortis/vo` (vx, vy only; yaw stays IMU-owned) from `fortis_perception`'s `rgbd_vo_node` — start that node separately. |

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | subscribed (wheel odometry) | wheel shaft velocities in rad/s from `joint_state_broadcaster` (`fortis_control`); read by joint name, never by index |
| `/odom` | `nav_msgs/Odometry` | published (wheel odometry) | body twist + dead-reckoned pose, frames `odom` -> `base_link`. The EKF fuses the TWIST only; the pose covariance is reported large and fixed because open-loop dead reckoning is never trustworthy on its own. |
| `<imu_topic>` | `sensor_msgs/Imu` | subscribed (debias node) | raw IMU stream |
| `/fortis/drive/armed` | `std_msgs/Bool` | subscribed (debias node) | latched (TRANSIENT_LOCAL); gates bias estimation |
| `/imu/debiased` | `sensor_msgs/Imu` | published (debias node) | copy of the input with the bias removed on `debias_axes`; header, frame_id, orientation, and all covariances pass through unchanged |
| `/odometry/filtered` | `nav_msgs/Odometry` | published (EKF) | the fused estimate; consumed by `fortis_drive/heading_hold_node` among others |

## Parameters (`imu_gyro_debias_node`)

| Param | Default | Meaning |
|---|---|---|
| `raw_imu_topic` | `/oak_chassis_front/imu/data` | Input topic (the launch file sets this from `imu_topic`). |
| `debiased_imu_topic` | `/imu/debiased` | Output topic. Fixed by the launch file — it is internal plumbing between the debias node and the EKF, so it is deliberately not a launch arg. |
| `ema_alpha` | `0.02` | Bias EMA rate. Slow by design: the bias is quasi-static, so a small alpha averages out per-sample gyro noise and only tracks slow thermal drift. |
| `debias_axes` | `["z"]` | Which `angular_velocity` axes to debias. Default z only: yaw rate is the sole EKF yaw input, and `two_d_mode` discards x/y rates anyway. |
| `settle_skip_s` | `0.5` | Ignore samples for this long after each arm->disarm edge, so the mechanical settle transient does not poison the bias EMA. |

`wheel_odometry_node` declares no parameters; the joint names, frames,
covariance values, and the max integration gap are module-level constants
(see the docstrings in `wheel_odometry_node.py` for the per-value rationale).

## Design decisions (the WHY)

- **IMU owns yaw.** The RR/RL wheels slip under load, so the wheel-derived
  yaw rate is the least trustworthy channel on the robot — a slipping rear
  wheel injects a phantom yaw rate the gyro never sees. `ekf.yaml` fuses the
  wheel twist as vx/vy ONLY (a loose translation hint) and the IMU gyro yaw
  rate as the SOLE yaw input. This is also what makes heading hold viable.
- **Velocities, not poses, are fused.** Wheel (and VO) pose is open-loop dead
  reckoning with no absolute reference; fusing an absolute pose would inject
  unbounded drift as if it were a measurement.
- **The EKF is the sole owner of `odom -> base_link`.** `wheel_odometry_node`
  publishes the `/odom` message but never TF (`ekf.yaml` sets
  `publish_tf: true`).
- **Stamp-driven time.** The odometry integrator and the heading-relevant dt
  logic use message header stamps, not wall clock, so behaviour stays correct
  under delayed, bursty, or bag-replayed input. Stamp gaps beyond 0.5 s
  re-anchor the clock instead of integrating across them (no pose teleports).
- **Stationary = DISARMED, not "measured velocity ~ 0".** Bias-driven creep
  keeps the wheels turning, so a velocity-based stillness test could never
  fire while the very drift being cancelled is active. Disarmed motors are
  the one trustworthy stillness signal.
- **`ekf.yaml` and `ekf_vio.yaml` stay in lockstep.** The VIO file is the
  base file plus the `odom1` (`/fortis/vo`) block, nothing else; a test
  enforces this.

## Testing

```bash
cd /workspace
colcon build --packages-select fortis_comms fortis_localization
source install/setup.bash
colcon test --packages-select fortis_localization
colcon test-result --verbose
```

- `test/test_wheel_odometry.py` — pure math (name-keyed wheel mapping, FK
  recovery, exact-integration pose updates, dt guards) plus a DDS round trip
  for the Odometry contract.
- `test/test_imu_gyro_debias.py` — bias EMA convergence, armed freeze, settle
  skip, metadata pass-through, plus a DDS round trip.
- `test/test_localization_launch.py` — both EKF YAMLs parse, the VIO variant
  fuses `/fortis/vo` velocity-only and preserves the wheel/IMU channels, and
  the launch file compiles and declares `vio`.

Tests pin `ROS_DOMAIN_ID=95` via `test/conftest.py` (the per-package
test-domain registry) so parallel `colcon test` processes cannot cross-talk.
