# fortis_bringup

Top-level launch composition for FORTIS. `bringup.launch.py` composes the mission state machine, the drive node, and the ODrive health monitor; `sim.launch.py` and `teleop.launch.py` are now live with concrete payloads (URDF publishing + Foxglove bridge, and keyboard teleop respectively). More node includes are added per package as each one comes online.

## Launch files

| File | Status | Purpose |
|---|---|---|
| `launch/bringup.launch.py` | live | Composes `mission_state_node` (`fortis_safety`), `drive_node` (`fortis_drive`), and `odrive_health_monitor_node` (`fortis_safety`). Optionally includes `fortis_localization`'s `localization.launch.py` (wheel odometry + EKF) behind `localization:=true` (default off). Arm controller, perception, and diagnostics are added as those packages come online. |
| `launch/sim.launch.py` | live | Brings up `robot_state_publisher` (xacro-expanded URDF on `/robot_description` + `/tf`), `joint_state_publisher` (gated by `publish_joint_states:=true|false` so an external simulator can take over `/joint_states`), and `foxglove_bridge` on a configurable port (default 8765). Designed to pair with Isaac Sim running on the Windows host outside the container. |
| `launch/teleop.launch.py` | live | Brings up `teleop_twist_keyboard` and remaps its output to `/cmd_vel`. Developer / debug-only entry point — production operator UI is the Foxglove + click-to-3D path. Must be launched in the foreground (the node reads stdin directly). |
| `launch/oak_chassis_cameras.launch.py` | live | **Primary camera bring-up.** Discovers every connected OAK-D Lite and starts each (serial-pinned front/rear/left/right) as an independent depthai-ros v3 `Driver`: on-device MJPEG RGB + aligned 16UC1 depth (no RGBD cloud), IMU on. Shared capture config `config/oak_chassis_cameras.yaml`. Load `foxglove/fortis_chassis_cams.json` (one camera per tab; only the active tab streams). |
| `launch/oak_chassis_front.launch.py` | live | Single-camera **front** debug bring-up (node `oak_chassis_front`, parent TF `front_camera_link`, plus a `front_camera_link -> oak_chassis_front` static TF). Same depthai-ros v3 `Driver` and the same `config/oak_chassis_cameras.yaml` as the multi-cam launch, for one camera. `localization.launch.py` relies on this launch for the front camera TF chain. |
| `launch/chassis_orbit.launch.py` | live | One-command orbit demo: includes `drive_test.launch.py` (`orbit:=true`) **+** `oak_chassis_cameras.launch.py` — all four cameras, the full drive stack, `orbit_node`, and one `foxglove_bridge`. Hold an ORBIT button in Foxglove to face-center orbit. |

## Config

| File | Purpose |
|---|---|
| `config/bringup_params.yaml` | Per-node parameter blocks loaded by `bringup.launch.py`. Forward-looking — most parameters are documented placeholders that the nodes have not yet adopted via `declare_parameter()`. See the file header for the registry rationale. |
| `config/oak_chassis_cameras.yaml` | Shared depthai-ros v3 capture config for ALL four chassis OAK-D Lites (identical units). 640x400 @ 15 fps, MJPEG-compressed RGB + aligned 16UC1 depth, no RGBD cloud, NN off (`i_nn_type: none`), IMU on. Loaded node-agnostically via `fortis_bringup.camera_params.load_camera_params()`, so it is tied to no single camera (the `/**` top key is a positional placeholder, read by position). |

## Building

```bash
colcon build --packages-select fortis_bringup
source install/setup.bash
```

## Running

```bash
ros2 launch fortis_bringup bringup.launch.py    # mission_state_node + drive_node + odrive_health_monitor_node
ros2 launch fortis_bringup bringup.launch.py localization:=true  # + wheel odometry + robot_localization EKF
ros2 launch fortis_bringup sim.launch.py        # robot_state_publisher + joint_state_publisher + foxglove_bridge
ros2 launch fortis_bringup teleop.launch.py     # teleop_twist_keyboard -> /cmd_vel
ros2 launch fortis_bringup oak_chassis_cameras.launch.py  # ALL connected OAK-D Lites: MJPEG RGB + depth + IMU (primary)
ros2 launch fortis_bringup oak_chassis_front.launch.py    # single front OAK-D Lite (debug)
ros2 launch fortis_bringup chassis_orbit.launch.py        # orbit demo: 4 cameras + drive stack + orbit + bridge
```

Live OAK streaming is intended for the Jetson over native USB; on Windows/WSL2 docker an OAK USB re-enumeration issue currently blocks it.

## TODO

- Add the arm controller include (`fortis_arm/arm_controller_node`) to `bringup.launch.py`.
- Add MoveIt 2 launch include once `fortis_moveit_config` exists.
- All four chassis OAK-D Lites already come up (serial-pinned) via `oak_chassis_cameras.launch.py`. Per the BOM they are 4x OAK-D Lite (Active Focus) A00483 (front + rear + left + right); the remaining 1x OAK-D Pro (Active Stereo IR, Active Focus, Standard FOV) A00546 on the arm L4 midpoint is added once `fortis_perception` exists.
- ~~Add `robot_localization` ekf_node once `fortis_localization` exists.~~ Done: `fortis_localization` provides `wheel_odometry_node` + the EKF via `localization.launch.py`, included by `bringup.launch.py` behind the `localization:=true` arg (default off).
- Add namespace + `use_sim_time` arguments to `sim.launch.py`.
- Add the upstream-to-FORTIS ODrive health bridge (translates `/odrive_status` into `fortis_msgs/OdriveHealth` for `odrive_health_monitor_node` to consume) once `ros_odrive` is vcs-imported.
