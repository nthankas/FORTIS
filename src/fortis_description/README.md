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

26 links, 25 joints. Single root `base_link`. No loop closures, no orphans.

| Type | Count |
|---|---|
| fixed | 15 |
| continuous | 4 (wheels) |
| revolute | 4 (J1, J2, J3, J4 with limits) |
| prismatic | 2 (gripper driven + mimic) |

## Source-of-truth dimensions

Arm chain is fully in-line: every joint axis sits on the arm's local X plane,
no lateral stow offsets. Tube lengths are slightly shorter than axis-to-axis
distances to leave room for motor housings at each end.

| Property | Value | Source |
|---|---|---|
| Chassis skeleton | 0.332 × 0.217 × 0.152 m | locked memory |
| Belly clearance | 0.051 m | locked memory |
| Wheel diameter | 0.203 m | AndyMark am-0463 |
| Wheel center positions (X) | ±0.176 m | CAD |
| Wheel center positions (Y) | ±0.125 m | CAD |
| J1 axis → J2 axis | 0.0559 m (2.199 in) | CAD |
| J2 axis → J3 axis | 0.3939 m (15.508 in) | CAD |
| J3 axis → J4 axis | 0.2992 m (11.781 in) | CAD |
| J4 axis → gripper base mount (X / Z) | 0.0635 m / 0.02286 m (2.5 in / 0.9 in) | CAD |
| Gripper base plate | 0.1041 × 0.060 × 0.010 m | ServoCity spec |
| Jaw stroke (per finger) | 0.027 m | ServoCity spec |
| Finger length (approach) | 0.058 m | ServoCity spec |
| j2_link CF tube length | 0.350 m (visual only) | CAD |
| j3_link CF tube length | 0.208 m (visual only) | CAD |
| J1 mount on chassis (X) | +0.0898 m | locked memory |
| Chassis camera height (all 4) | 0.21514 m (8.47 in) above ground | CAD |
| Chassis camera edge gap | 0.01933 m (0.761 in) from chassis edge to housing | CAD |

## Reach (computed forward kinematics, all joints at zero)

| Measurement | Value |
|---|---|
| J2 axis to gripper TCP (horizontal) | 0.802 m (31.6 in) |

Still clears the 30" reach criterion in Success Criteria Matrix V4.0 (T2).

## Joint limits

| Joint | Lower (rad / m) | Upper (rad / m) | Effort | Velocity |
|---|---|---|---|---|
| joint_j1 | -2.967 (-170°) | +2.967 (+170°) | 15.0 Nm | 1.0 rad/s |
| joint_j2 | -1.571 (-90°)  | +1.571 (+90°)  | 40.0 Nm | 1.0 rad/s |
| joint_j3 | -2.618 (-150°) | +2.618 (+150°) | 15.0 Nm | 1.0 rad/s |
| joint_j4 | -1.571 (-90°)  | +1.571 (+90°)  | 3.5 Nm  | 1.0 rad/s |
| gripper_left_joint | 0.0 m | 0.027 m | 10.0 N | 0.05 m/s |

`gripper_right_joint` mirrors `gripper_left_joint` via a `<mimic>` tag with
multiplier=-1. Only `gripper_left_joint` is commandable.

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
- `arm_mount`, `arm_base`, `link1`..`link4`
- `arm_camera_link`, `arm_camera_optical_frame`
- `gripper_base`, `gripper_left_finger`, `gripper_right_finger`
- `gripper_tcp` (tool center point for MoveIt grasp planning)

Optical frames follow ROS REP-103 (Z forward, X right, Y down) so they
plug directly into `depthai_ros_driver_v3` and `image_pipeline` without
remapping.

## ros2_control wiring

`urdf/fortis_chassis.ros2_control.xacro` contains the `<ros2_control>`
system block for the four wheels. It is included from
`fortis_robot.urdf.xacro` only when the `enable_ros2_control:=true`
xacro arg is passed. `display.launch.py` (RViz only) does not pass it,
so the legacy RViz workflow keeps working unchanged. The
`fortis_control` package's launch files pass it to drive real motors
via `odrive_ros2_control`.

## Next steps (not in this package)

1. MoveIt 2 setup_assistant: point at the expanded URDF, generate
   `fortis_moveit_config`. Self-collision matrix will be quick because
   the arm chain is clean.
2. Arm `<ros2_control>` block (Teensy SystemInterface). Sibling xacro
   to `fortis_chassis.ros2_control.xacro`; deferred until the Teensy
   serial bridge node lands and the protocol can be plumbed to a
   real `hardware_interface::SystemInterface`.
3. `robot_localization` ekf_node: configure with wheel odometry +
   `arm_camera_optical_frame` VIO from cuVSLAM / Basalt.
