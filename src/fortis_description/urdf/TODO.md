# urdf/

URDF / xacro source for the FORTIS robot.

**Currently empty.** Blocked on the OnShape export (Adrian + Carlos own this).

## What goes here

- `fortis.urdf.xacro` — root xacro, includes per-subassembly macros below.
- `chassis.xacro` — chassis links (`base_link`, four `*_wheel_link`s, mounted-camera links).
- `arm.xacro` — 4-joint arm chain (`arm_base_link` -> `j1_link` -> ... -> `j4_link`) plus gripper.
- `materials.xacro` — shared `<material>` definitions.
- `inertial.xacro` — shared mass / inertia macros.

Frame and joint names are fixed (see `CLAUDE.md` -> "Frame naming conventions" and the integration test's `fortis_safety` / `fortis_drive` constants). Do not rename frames without updating every consumer in lockstep.

## Interim status

Until the real URDF lands, downstream packages should depend on `fortis_description` only when they explicitly need a robot model (RViz, MoveIt config, robot_state_publisher). Nothing in this package is loaded automatically.

## TODO

- [ ] Receive OnShape URDF export (Adrian + Carlos).
- [ ] Split into per-subassembly xacro macros above.
- [ ] Add gazebo / ros2_control xacro tags once those packages need them.
- [ ] Wire `display.launch.py` to publish `robot_description`, start `robot_state_publisher`, and open RViz with `rviz/fortis.rviz`.
