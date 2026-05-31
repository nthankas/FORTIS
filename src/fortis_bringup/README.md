# fortis_bringup

Top-level launch composition for FORTIS. `bringup.launch.py` composes the mission state machine, the drive node, and the ODrive health monitor; `sim.launch.py` and `teleop.launch.py` are now live with concrete payloads (URDF publishing + Foxglove bridge, and keyboard teleop respectively). More node includes are added per package as each one comes online.

## Launch files

| File | Status | Purpose |
|---|---|---|
| `launch/bringup.launch.py` | live | Composes `mission_state_node` (`fortis_safety`), `drive_node` (`fortis_drive`), and `odrive_health_monitor_node` (`fortis_safety`). Arm controller, perception, localization, and diagnostics are added as those packages come online. |
| `launch/sim.launch.py` | live | Brings up `robot_state_publisher` (xacro-expanded URDF on `/robot_description` + `/tf`), `joint_state_publisher` (gated by `publish_joint_states:=true|false` so an external simulator can take over `/joint_states`), and `foxglove_bridge` on a configurable port (default 8765). Designed to pair with Isaac Sim running on the Windows host outside the container. |
| `launch/teleop.launch.py` | live | Brings up `teleop_twist_keyboard` and remaps its output to `/cmd_vel`. Developer / debug-only entry point — production operator UI is the Foxglove + click-to-3D path. Must be launched in the foreground (the node reads stdin directly). |
| `launch/oak_chassis_front.launch.py` | live | Includes the upstream `depthai_ros_driver` `camera.launch.py` for the single **front** OAK-D Lite (node name `oak_chassis_front`, parent TF `front_camera_link`). Publishes RGB, depth-aligned stereo, a colored RGBD point cloud, and raw IMU. Rear / left / right cameras and any VIO / localization are out of scope here. Pair with `sim.launch.py` (or a standalone `robot_state_publisher` + `foxglove_bridge`) for the URDF TF tree and a viewer; load `foxglove/fortis_chassis_cam.json`. |

## Config

| File | Purpose |
|---|---|
| `config/bringup_params.yaml` | Per-node parameter blocks loaded by `bringup.launch.py`. Forward-looking — most parameters are documented placeholders that the nodes have not yet adopted via `declare_parameter()`. See the file header for the registry rationale. |
| `config/oak_chassis_front.yaml` | `depthai_ros_driver` params for the front OAK-D Lite. Pins the physical camera by MXID (`i_mx_id`) so it is selected deterministically among the four identical units; 1080p RGB, 480p depth-aligned subpixel stereo, NN off (`i_nn_type: none`) until a model is trained. The top-level key must equal the launch `name` (`oak_chassis_front`) or the driver ignores the file. |

## Building

```bash
colcon build --packages-select fortis_bringup
source install/setup.bash
```

## Running

```bash
ros2 launch fortis_bringup bringup.launch.py    # mission_state_node + drive_node + odrive_health_monitor_node
ros2 launch fortis_bringup sim.launch.py        # robot_state_publisher + joint_state_publisher + foxglove_bridge
ros2 launch fortis_bringup teleop.launch.py     # teleop_twist_keyboard -> /cmd_vel
ros2 launch fortis_bringup oak_chassis_front.launch.py  # front OAK-D Lite: RGB + depth + RGBD cloud + IMU
```

Live OAK streaming is intended for the Jetson over native USB; on Windows/WSL2 docker an OAK USB re-enumeration issue currently blocks it.

## TODO

- Add the arm controller include (`fortis_arm/arm_controller_node`) to `bringup.launch.py`.
- Add MoveIt 2 launch include once `fortis_moveit_config` exists.
- Add the remaining `depthai_ros_driver` includes once `fortis_perception` exists. The **front** OAK-D Lite is already brought up by `oak_chassis_front.launch.py`; per the BOM the rest are 3x more OAK-D Lite (Active Focus) A00483 (left + right toroidal VIO, rear outward depth) plus 1x OAK-D Pro (Active Stereo IR, Active Focus, Standard FOV) A00546 on the arm L4 midpoint.
- Add `robot_localization` ekf_node once `fortis_localization` exists.
- Add namespace + `use_sim_time` arguments to `sim.launch.py`.
- Add the upstream-to-FORTIS ODrive health bridge (translates `/odrive_status` into `fortis_msgs/OdriveHealth` for `odrive_health_monitor_node` to consume) once `ros_odrive` is vcs-imported.
