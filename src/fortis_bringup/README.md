# fortis_bringup

Top-level launch composition for FORTIS. `bringup.launch.py` composes the mission state machine, the drive stack (drive node + UI enable/disable), and the ODrive health monitor, with opt-in includes for localization, heading hold, the orbit generator, the perception chain, and the arm seam. `sim.launch.py` publishes the URDF + a Foxglove bridge for external-simulator runs; `teleop.launch.py` is keyboard teleop for developer iteration.

## Launch files

| File | Status | Purpose |
|---|---|---|
| `launch/bringup.launch.py` | live | Composes `mission_state_node` + `odrive_health_monitor_node` (`fortis_safety`) and `drive_node` + `drive_enable_node` (`fortis_drive`). Opt-ins, all default off: `localization:=true` adds `fortis_localization`'s `localization.launch.py` (wheel odometry + EKF); `heading_hold:=true` runs `heading_hold_node` and feeds `drive_node` from `/cmd_vel_heading` (closed-loop yaw; enable alongside `localization:=true`); `orbit:=true` runs `orbit_node` (the `chassis_orbit` launch sets it); `perception:=true` adds the full perception chain via `perception.launch.py` (including its default-on `foxglove_bridge` — bringup runs none of its own); `arm:=true` adds `teensy_bridge` + `arm_controller` (`fortis_arm`; Teensy port via `serial_port`, default `/dev/ttyACM0`). |
| `launch/sim.launch.py` | live | Brings up `robot_state_publisher` (xacro-expanded URDF on `/robot_description` + `/tf`), `joint_state_publisher` (gated by `publish_joint_states:=true|false` so an external simulator can take over `/joint_states`), and `foxglove_bridge` on a configurable port (default 8765). Designed to pair with Isaac Sim running on the Windows host outside the container. |
| `launch/teleop.launch.py` | live | Brings up `teleop_twist_keyboard` and remaps its output to `/cmd_vel`. Developer / debug-only entry point — production operator UI is the Foxglove + click-to-3D path. Must be launched in the foreground (the node reads stdin directly). |
| `launch/oak_chassis_cameras.launch.py` | live | **Primary camera bring-up.** Discovers every connected OAK-D Lite and starts each (serial-pinned front/rear/left/right) as an independent depthai-ros v3 `Driver`: on-device MJPEG RGB + aligned 16UC1 depth (no RGBD cloud), IMU on. Each roster camera also gets a static identity TF `<pos>_camera_link -> oak_chassis_<pos>` attaching its calibration TF tree to the URDF (unknown serials stay detached). Shared capture config `config/oak_chassis_cameras.yaml`. Load `foxglove/fortis_chassis_cams.json` (one camera per tab; only the active tab streams). |
| `launch/oak_chassis_front.launch.py` | live | Single-camera **front** debug bring-up (node `oak_chassis_front`, parent TF `front_camera_link`, plus a `front_camera_link -> oak_chassis_front` static TF). Same depthai-ros v3 `Driver` and the same `config/oak_chassis_cameras.yaml` as the multi-cam launch, for one camera. `localization.launch.py` relies on this launch (or the multi-cam launch) for the front camera TF chain. |
| `launch/chassis_orbit.launch.py` | live | One-command orbit demo: includes `drive_test.launch.py` (`orbit:=true`) **+** `oak_chassis_cameras.launch.py` — all four cameras, the full drive stack, `orbit_node`, and one `foxglove_bridge`. Hold an ORBIT button in Foxglove to face-center orbit. |
| `launch/perception.launch.py` | live | **Perception demo entry point.** Synthetic-or-real camera source -> per-camera point clouds -> fused cloud -> voxel map (+ optional cross-run diff) -> detection -> click-to-target + system health, optional RGBD VO, one optional `foxglove_bridge` (:8765), optional arm seam. See "Perception bring-up" below. |

## Config

| File | Purpose |
|---|---|
| `config/bringup_params.yaml` | Per-node parameter blocks loaded by `bringup.launch.py`. Mixed live/documentation: the `heading_hold_node`, `orbit_node`, and `imu_gyro_debias_node` blocks are LIVE overrides (those nodes `declare_parameter()` every value); the remaining blocks document in-code constants the nodes do not read yet. See the file header for which is which. |
| `config/oak_chassis_cameras.yaml` | Shared depthai-ros v3 capture config for ALL four chassis OAK-D Lites (identical units). 640x400 @ 15 fps, MJPEG-compressed RGB + aligned 16UC1 depth, no RGBD cloud, NN off (`i_nn_type: none`), IMU on. Loaded node-agnostically via `fortis_bringup.camera_params.load_camera_params()`, so it is tied to no single camera (the `/**` top key is a positional placeholder, read by position). |

## Building

```bash
colcon build --packages-select fortis_bringup
source install/setup.bash
```

## Running

```bash
ros2 launch fortis_bringup bringup.launch.py    # FSM + drive_node + drive_enable_node + ODrive health monitor
ros2 launch fortis_bringup bringup.launch.py localization:=true  # + wheel odometry + robot_localization EKF
ros2 launch fortis_bringup bringup.launch.py perception:=true    # + full perception chain (and its foxglove_bridge)
ros2 launch fortis_bringup bringup.launch.py arm:=true           # + teensy_bridge + arm_controller (serial_port:=...)
ros2 launch fortis_bringup sim.launch.py        # robot_state_publisher + joint_state_publisher + foxglove_bridge
ros2 launch fortis_bringup teleop.launch.py     # teleop_twist_keyboard -> /cmd_vel
ros2 launch fortis_bringup oak_chassis_cameras.launch.py  # ALL connected OAK-D Lites: MJPEG RGB + depth + IMU (primary)
ros2 launch fortis_bringup oak_chassis_front.launch.py    # single front OAK-D Lite (debug)
ros2 launch fortis_bringup chassis_orbit.launch.py        # orbit demo: 4 cameras + drive stack + orbit + bridge
ros2 launch fortis_bringup perception.launch.py           # synthetic perception demo (see "Perception bring-up")
```

Live OAK streaming is intended for the Jetson over native USB; on Windows/WSL2 docker an OAK USB re-enumeration issue currently blocks it.

## Perception bring-up

`perception.launch.py` composes the perception chain end to end. With the default `synthetic:=true` it needs no hardware: one `oak_replayer` (`fortis_sim_support`) orbits the synthetic scene and publishes the exact front-camera topic contract. The mission FSM is NOT started here — run `bringup.launch.py perception:=true` for the full click-to-TARGETING flow.

| Arg | Default | Purpose |
|---|---|---|
| `synthetic` | `true` | `true`: one `oak_replayer` (orbit trajectory, TF on) stands in for the front camera. `false`: includes `oak_chassis_cameras.launch.py` (real OAKs). |
| `scene` | `baseline` | Synthetic scene variant: `baseline` \| `modified` (one box added, one removed). |
| `cameras` | `oak_chassis_front` | Comma list: one `depth_to_cloud` per name and the `cloud_fusion` input set. The first name is the detection / VO / replayer camera. |
| `detector` | `blob` | `blob` (HSV, no weights — the synthetic/CI backend) \| `yolo` (needs `ros2 run fortis_perception download_models`). |
| `vio` | `true` | Run `rgbd_vo` (`/fortis/vo`). The EKF that fuses it is owned by `bringup.launch.py` (`localization:=true vio:=true` selects `ekf_vio.yaml`). |
| `reference_map` | `""` | Path to a saved `.npz` voxel map; non-empty starts `map_diff` against it. |
| `foxglove` | `true` | Run the single `foxglove_bridge` on :8765. Set `false` when composing with a launch that already owns a bridge (`drive_test` / `chassis_orbit`). |
| `arm` | `false` | Run `teensy_bridge` + `arm_controller` (`fortis_arm`). |
| `serial_port` | `/dev/ttyACM0` | Teensy USB-CDC port for `teensy_bridge`; use the pty printed by `tools/mock_teensy.py` for hardware-free runs. |

Synthetic demo one-liner (view at `ws://<host>:8765` in Foxglove):

```bash
ros2 launch fortis_bringup perception.launch.py
```

Two-run map-diff workflow (the cross-run change detector):

```bash
# Run 1 -- baseline scene; let the map accumulate for a minute, then save it:
ros2 launch fortis_bringup perception.launch.py scene:=baseline
ros2 service call /voxel_map/save_map fortis_msgs/srv/SaveMap "{path: '/tmp/baseline.npz'}"

# Run 2 -- restart against the modified scene with the saved reference:
ros2 launch fortis_bringup perception.launch.py scene:=modified reference_map:=/tmp/baseline.npz
# Watch /fortis/perception/map_diff/markers (green = added, red = removed)
# and /fortis/perception/map_diff/summary for the voxel counts.
```

## TODO

- ~~Add the arm controller include (`fortis_arm/arm_controller_node`) to `bringup.launch.py`.~~ Done: `arm:=true` launches `teensy_bridge` + `arm_controller` (and `perception.launch.py` carries its own `arm` arg for standalone runs).
- Add MoveIt 2 launch include once `fortis_moveit_config` exists.
- All four chassis OAK-D Lites already come up (serial-pinned) via `oak_chassis_cameras.launch.py`. Per the BOM they are 4x OAK-D Lite (Active Focus) A00483 (front + rear + left + right); the remaining 1x OAK-D Pro (Active Stereo IR, Active Focus, Standard FOV) A00546 on the arm L4 midpoint is added once `fortis_perception` exists.
- ~~Add `robot_localization` ekf_node once `fortis_localization` exists.~~ Done: `fortis_localization` provides `wheel_odometry_node` + the EKF via `localization.launch.py`, included by `bringup.launch.py` behind the `localization:=true` arg (default off).
- Add namespace + `use_sim_time` arguments to `sim.launch.py`.
- Add the upstream-to-FORTIS ODrive health bridge (translates `/odrive_status` into `fortis_msgs/OdriveHealth` for `odrive_health_monitor_node` to consume) once `ros_odrive` is vcs-imported.
