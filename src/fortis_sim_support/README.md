# fortis_sim_support

Hardware-free synthetic data sources for FORTIS: a procedural RGBD test
scene, a vectorized raycaster, closed-form trajectories, and an OAK camera
replayer that publishes the exact depthai v3 topic contract of the real
chassis cameras.

**This package is the hardware-free test path for all perception work.** The
OAK-D cameras cannot stream inside the WSL2 dev container, so every
`fortis_perception` node must be runnable and testable against these synthetic
sources first; live-camera runs on the Jetson are the final check, not the
development loop.

## Modules

| Module | Role |
|---|---|
| `synthetic_scene.py` | Pure-numpy scene: ground plane + axis-aligned boxes + spheres. Every material combines a base RGB, a world-space checkerboard, and deterministic per-primitive RNG speckle, so every surface has ORB-trackable texture (see "Why the speckle is anti-aliased" below). `scene_baseline()` / `scene_modified()` differ by exactly `ADDED_BOX` (0.4 m cube, 0.064 m³) and `REMOVED_BOX` (0.35 m cube, 0.042875 m³) — both module constants, so map-diff tests can assert exact volumes. |
| `raycaster.py` | Vectorized ray/plane, ray/AABB (slab), ray/sphere. `render(scene, K, T_world_cam, w, h, max_range=8.0)` → `(rgb uint8 HxWx3, depth uint16 HxW mm)`. Depth is planar camera-Z distance (matching the OAK's rgb-aligned depth); `0` = no return. Ray grids are `lru_cache`d per (K, size). Pure math, no ROS imports. |
| `trajectory.py` | Closed-form `hold` / `line` / `orbit` for base_link in the odom frame. `sample(t)` returns exact `(xyz, quat_xyzw, lin_vel_world, ang_vel_world)` — analytic ground truth for VIO/mapping error bounds. |
| `oak_replayer_node.py` | Node `oak_replayer`: renders the scene along the trajectory and publishes the camera topic contract below for ONE chassis mount, plus (on the rig's designated owner) `/fortis/sim/ground_truth` and the base TF. |

## Topic contract (per camera, matches depthai-ros v3)

```
/<camera_name>/rgb/image_raw/compressed   sensor_msgs/CompressedImage (jpeg)
/<camera_name>/rgb/camera_info            sensor_msgs/CameraInfo
/<camera_name>/stereo/image_raw           sensor_msgs/Image (16UC1, aligned to rgb)
/<camera_name>/stereo/camera_info         sensor_msgs/CameraInfo (same K as rgb)
/<camera_name>/imu/data                   sensor_msgs/Imu
/fortis/sim/ground_truth                  nav_msgs/Odometry (odom -> base_link, exact)
```

Image and depth share `frame_id = <camera_name>_rgb_camera_optical_frame`
(aligned depth lives in the rgb frame, as with `stereo.i_aligned` on the real
driver). The IMU uses `<camera_name>_imu_frame`.

## Mount presets and multi-camera rigs

The `mount` parameter picks which of the four chassis-camera mounts this
instance replays -- `front | rear | left | right`, with the URDF poses from
`fortis_description` hardcoded in `oak_replayer_node.MOUNTS` (front sits on
the base_link **-X** face, pitched 30 deg up; the others are level and face
outward from their face). `camera_name` defaults to `oak_chassis_<mount>`,
so topic namespaces match the real rig with no extra configuration.

Every instance samples the SAME base trajectory -- one rigid robot, up to
four cameras. Exactly one instance per rig may therefore own the dynamic
`odom -> base_link` TF and `/fortis/sim/ground_truth`: `publish_base_tf`
(default `true`) gates both, and the multi-camera launch enables it on the
FIRST replayer only. With `publish_tf` every instance still broadcasts its
own static `base_link -> <camera_name>` mount TF plus
`<camera_name> -> <camera_name>_rgb_camera_optical_frame` (the live rig's
intermediate `<mount>_camera_link` is collapsed into the mount transform
here), so the TF chain matches the live bring-up.

## Parameters (`oak_replayer`)

| Parameter | Default | Meaning |
|---|---|---|
| `mount` | `front` | Chassis mount to replay: `front` \| `rear` \| `left` \| `right` (URDF pose from MOUNTS). |
| `camera_name` | `""` | Topic namespace and frame-name prefix; empty derives `oak_chassis_<mount>`. |
| `width` / `height` | `640` / `400` | Render resolution (matches the real capture config). |
| `fps` | `15.0` | RGBD frame rate. |
| `imu_rate_hz` | `100.0` | IMU sample rate. |
| `trajectory` | `orbit` | `orbit` \| `line` \| `hold`. |
| `orbit_radius` | `1.0` | Orbit radius (m); also the start pose for `hold` / `line`. |
| `orbit_omega` | `0.3` | Orbit angular rate (rad/s); sign picks the direction. |
| `scene` | `baseline` | `baseline` \| `modified` (ADDED_BOX in, REMOVED_BOX out). |
| `publish_tf` | `true` | Broadcast the static camera TFs (+ odom->base_link on the owner). |
| `publish_base_tf` | `true` | Own the dynamic odom->base_link TF + ground truth. Exactly one instance per rig. |
| `imu_gyro_bias_z` | `0.0` | Injected z-gyro bias (rad/s), for debias testing. |
| `imu_noise_std` | `0.0` | Gaussian gyro noise sigma (rad/s). |
| `jpeg_quality` | `60` | RGB JPEG encode quality. |
| `seed` | `0` | RNG seed for the IMU noise stream (output is deterministic). |

## Launch

```bash
# Default: baseline scene, 1 m orbit, front mount only, capture-config rates.
ros2 launch fortis_sim_support synthetic_oak.launch.py

# Full 4-camera rig (the first replayer owns base TF + ground truth).
ros2 launch fortis_sim_support synthetic_oak.launch.py cameras:=front,rear,left,right

# Cross-run diff source: same orbit, modified scene.
ros2 launch fortis_sim_support synthetic_oak.launch.py scene:=modified

# Static low-rate debug feed without TF.
ros2 launch fortis_sim_support synthetic_oak.launch.py \
    trajectory:=hold fps:=5.0 publish_tf:=false
```

## Why the speckle is anti-aliased (it matters for VO)

The RNG speckle exists to give every surface ORB-trackable features, but
point-sampled noise ALIASES once the surface footprint of one pixel exceeds
the noise cell: the same wall then renders a different speckle pattern from
every camera pose, and that view-dependent shimmer actively corrupts feature
matching -- VO looked broken when the bug was the texture. The raycaster
passes each hit's per-pixel surface footprint into `shade()`, which fades
every speckle octave out as footprint/cell grows -- a poor man's mip-map,
like a real slightly-soft camera. Two octaves (fine + 8x coarser) keep
surfaces trackable both up close and across the room, and because checker +
speckle are functions of world position only, a surface point always renders
the same color from any pose -- exactly what cross-frame feature tracking
needs.

## Conventions

- Orbit heading matches `fortis_drive`'s face-the-center semantics: the FORTIS
  front (**base_link -X**) points at the orbit center for all t, i.e. yaw(t)
  equals the robot's polar angle about the center.
- Trajectory time is `frame_index / rate`, not wall clock, so images and
  ground truth share exact sample times and runs are reproducible per seed.
- Any latched topic this package publishes must use
  `fortis_comms.qos_profiles.latched_qos_profile` (TRANSIENT_LOCAL +
  RELIABLE), same as the rest of the stack. (None currently: all streams
  here are periodic.)
- Tests pin `ROS_DOMAIN_ID=97` (see the registry in `test/conftest.py`).
