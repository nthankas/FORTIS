# fortis_sim_support

Hardware-free synthetic data sources for FORTIS: a procedural RGBD scene
renderer and an OAK camera replayer that publishes the exact depthai v3 topic
contract of the real chassis cameras.

**This package is the hardware-free test path for all perception work.** The
OAK-D cameras cannot stream inside the WSL2 dev container, so every
`fortis_perception` node must be runnable and testable against these synthetic
sources first; live-camera runs on the Jetson are the final check, not the
development loop.

## Planned modules

| Module | Role |
|---|---|
| `synthetic_scene.py` | Procedural 3D test scene; renders registered RGB + depth pairs with known ground truth. |
| `raycaster.py` | Pinhole-camera depth raycasting core (pure math, no ROS imports). |
| `trajectory.py` | Time-parameterised camera/robot trajectories; doubles as ground truth for odometry and mapping tests. |
| `oak_replayer_node.py` | Node `oak_replayer`: publishes synthetic/recorded frames under the depthai v3 topic contract, plus `/fortis/sim/ground_truth` (`nav_msgs/Odometry`). |

All modules are currently stubs; the table is the contract for what lands
where.

## Launch

```bash
ros2 launch fortis_sim_support synthetic_oak.launch.py camera_name:=oak_chassis_front
```

One `oak_replayer` node, keyed by `camera_name` to mirror the real camera
bring-up. The launch file will grow scene/trajectory arguments as the
renderer lands.

## Conventions

- Any latched topic this package publishes must use
  `fortis_comms.qos_profiles.latched_qos_profile` (TRANSIENT_LOCAL +
  RELIABLE), same as the rest of the stack.
- Tests pin `ROS_DOMAIN_ID=97` (see the registry in `test/conftest.py`).
