# fortis_description

Hand-written xacro description of the FORTIS robot. Built fresh from the
ROS_Expanded_Chassis CAD (treated as truth), not via the OnShape native
exporter. Uses primitive geometry only — no STL meshes.

## Why this exists

OnShape's native URDF exporter produces output unusable for MoveIt /
ros2_control: closed kinematic loops with orphan loop-closure links, no
collision geometry, no joint limits, continuous joints where revolute is
required, spurious prismatic mate constraints, and asymmetric gripper
drive paths. See chat history for the full audit.

This package replaces all of that with a clean tree of primitives.

## Contents

```
fortis_description/
  urdf/
    fortis_constants.xacro     all dimensions in one place
    fortis_inertia.xacro       box/cylinder/sphere/tube inertia macros
    fortis_materials.xacro     RViz visual colors
    fortis_chassis.urdf.xacro  base_link, body, 4 wheels, 4 chassis cameras
    fortis_arm.urdf.xacro      4-DOF arm, gripper (1 driven + 1 mimic), arm cam
    fortis_robot.urdf.xacro    top-level, includes everything
  launch/
    display.launch.py          RViz + joint_state_publisher_gui
  rviz/
    fortis.rviz                minimal RViz config
  config/                      (reserved for ros2_control / MoveIt yamls)
  meshes/                      (reserved if visual STLs are added later)
  package.xml
  CMakeLists.txt
```

## Tree summary

25 links, 24 joints. Single root `base_link`. No loop closures, no orphans.

| Type | Count |
|---|---|
| fixed | 14 |
| continuous | 4 (wheels) |
| revolute | 4 (J1, J2, J3, J4 with limits) |
| prismatic | 2 (gripper driven + mimic) |

Total mass at default values: 12.71 kg.

## Source-of-truth dimensions

Extracted from ROS_Expanded_Chassis/urdf/ros_expanded_chassis.urdf, with
separate values for CF tube physical length and joint-axis-to-joint-axis
distance (the kinematic span). The two differ because motor housings,
gearbox flanges, and mounting brackets add length at each end of the CF
tube.

| Property | Value | Source |
|---|---|---|
| Chassis skeleton | 0.332 × 0.217 × 0.152 m | locked memory |
| Belly clearance | 0.051 m | locked memory |
| Wheel diameter | 0.203 m | AndyMark am-0463 |
| Wheel center positions (X) | ±0.176 m | CAD |
| Wheel center positions (Y) | ±0.125 m | CAD |
| J1 axis → J2 axis | 0.091 m (3.59 in) | CAD |
| J2 axis → J3 axis | 0.411 m (16.16 in) | CAD |
| J3 axis → J4 axis | 0.300 m (11.82 in) | CAD |
| J4 axis → EE attach | 0.068 m (2.68 in) | CAD |
| EE attach → gripper base | 0.030 m (1.20 in) | CAD |
| j2_link CF tube length | 0.360 m (visual only) | CAD |
| j3_link CF tube length | 0.208 m (visual only) | CAD |
| j4_link CF tube length | 0.038 m (visual only) | CAD |
| J1 mount on chassis (X) | +0.0898 m | locked memory (3 in from rear edge) |

## Reach (computed forward kinematics, all joints at zero)

| Measurement | Value |
|---|---|
| J1 axis to gripper tip | 1.020 m (40.16 in) |
| J2 axis to gripper tip | 0.929 m (36.57 in) |
| J2 axis to gripper base | 0.839 m (33.03 in) |

Clears the 30" reach criterion in Success Criteria Matrix V4.0 (T2).

## Joint limits

| Joint | Lower (rad) | Upper (rad) | Effort (Nm) | Velocity (rad/s) |
|---|---|---|---|---|
| joint_j1 | -2.967 (-170°) | +2.967 (+170°) | 15.0 | 1.0 |
| joint_j2 | -1.571 (-90°)  | +1.571 (+90°)  | 40.0 | 1.0 |
| joint_j3 | -2.618 (-150°) | +2.618 (+150°) | 15.0 | 1.0 |
| joint_j4 | -1.571 (-90°)  | +1.571 (+90°)  | 3.5  | 1.0 |
| joint_gripper | 0.0 m | 0.040 m | 20.0 | 0.05 |

Adjust in `urdf/fortis_constants.xacro`.

## Build and run

Drop this package under `src/` in your colcon workspace (so
`E:\Capstone\Projects\FORTIS\src\fortis_description\`), then:

```bash
cd ~/your_ws
colcon build --packages-select fortis_description
source install/setup.bash
ros2 launch fortis_description display.launch.py
```

To expand to plain URDF (for tooling that wants flat URDF):
```bash
ros2 run xacro xacro \
  $(ros2 pkg prefix fortis_description)/share/fortis_description/urdf/fortis_robot.urdf.xacro \
  > /tmp/fortis.urdf
check_urdf /tmp/fortis.urdf
```

## Frames published

- `base_link` (root, chassis center at ground level)
- `chassis_body`
- `fl_wheel`, `fr_wheel`, `rl_wheel`, `rr_wheel`
- `front_camera_link`, `front_camera_optical_frame` (and rear/left/right)
- `arm_mount`, `arm_base`, `link1`..`link4`, `ee`
- `arm_camera_link`, `arm_camera_optical_frame`
- `jaw_left`, `jaw_right`

Optical frames follow ROS REP-103 (Z forward, X right, Y down) so they
plug directly into `depthai_ros_driver_v3` and `image_pipeline` without
remapping.

## Next steps (not in this package)

1. `ros2_control` `<ros2_control>` block: add to a separate xacro that
   includes this one, wires up `odrive_ros2_control` for wheels and the
   custom Teensy SystemInterface for the arm.
2. MoveIt 2 setup_assistant: point at the expanded URDF, generate
   `fortis_moveit_config`. Self-collision matrix will be quick because
   the arm chain is clean.
3. `robot_localization` ekf_node: configure with wheel odometry +
   `arm_camera_optical_frame` VIO from cuVSLAM / Basalt.
