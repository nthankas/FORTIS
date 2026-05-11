# ADR 0001: Dual development container strategy (CPU + Isaac ROS GPU)

- **Status:** Accepted
- **Date:** 2026-05-10
- **Deciders:** FORTIS team

## Context

The FORTIS development team works on a mix of hardware:

- Linux/macOS laptops with no NVIDIA GPU (some teammates).
- FORTIS PC desktop, RTX 4080.
- IdeaPad laptop, RTX 5050 Laptop.
- Jetson Orin Nano Super (target deployment platform).

We want every teammate to be able to clone the repo, bring up a container,
build the workspace, and run unit tests without buying new hardware. We
*also* want the GPU-equipped machines to benefit from NVIDIA's Isaac ROS
packages, which provide high-value perception capabilities:

- `isaac_ros_visual_slam` (cuVSLAM) for visual-inertial odometry,
- `isaac_ros_nvblox` for GPU-accelerated 3D reconstruction,
- `isaac_ros_cumotion` and `isaac_ros_cumotion_moveit` for GPU motion planning,
- `isaac_ros_image_proc` and `isaac_ros_image_pipeline` for accelerated image
  processing.

These packages require an NVIDIA GPU plus the NVIDIA Container Toolkit at
runtime, and the Isaac ROS base image is multi-GB. Forcing all teammates to
pull and run that image would exclude the non-NVIDIA developers.

Isaac ROS 4.x dropped ROS 2 Humble in favor of Jazzy. The FORTIS stack is on
Humble (matched to Jetson JetPack 6 and to the rest of the team's tooling),
so the Isaac ROS path is pinned to the **3.2 release line** (`release-3.2`
branch / `*_ros2_humble_release-3.2` Docker tags), which is the last 3.x
release supporting Humble.

## Decision

Ship **two development containers** maintained side by side:

1. **`fortis-dev`** (`docker/Dockerfile.dev`, existing) - CPU-only, based on
   `osrf/ros:humble-desktop`. The default container. Builds on any x86_64 or
   aarch64 Docker host. Used by all teammates regardless of hardware.
2. **`fortis-dev-gpu`** (`docker/Dockerfile.dev-gpu`, new) - based on the
   NVIDIA Isaac ROS Dev Base for `ros2_humble_release-3.2` (multi-arch:
   `x86_64-` and `aarch64-` variants), with the Isaac ROS apt repository
   pre-configured and the perception/motion packages above pre-installed.
   Used only by teammates with an NVIDIA GPU.

Both containers:

- Install the same `ros-humble-moveit`, `ros2_control`, `robot_localization`,
  `foxglove_bridge`, `xacro` and other ROS apt deps.
- Install the same pip deps (`depthai`, `python-can`, `pyserial`, `odrive`,
  numpy / scipy / matplotlib).
- Mount the repository at `/workspace`.
- Create the same `FORTIS` user with passwordless sudo.

The GPU container is wired in as a Docker Compose **overlay**
(`docker/docker-compose.gpu.yml`) on top of the existing
`docker/docker-compose.yml`, so it inherits the volume mounts and ROS env
vars rather than duplicating them.

## Alternatives considered

1. **Single Isaac ROS container for everyone.** Rejected. Excludes non-NVIDIA
   teammates entirely. Even on NVIDIA hosts, the multi-GB image is heavy
   for CI and for laptops with limited storage. Also forces the Jazzy/Humble
   alignment problem onto every teammate even if they never touch Isaac ROS.
2. **Single CPU container for everyone.** Rejected. Throws away the entire
   reason FORTIS PC + Jetson exist on the project: GPU-accelerated VSLAM,
   nvblox reconstruction, cuMotion planning. We would lose the perception
   capability gap that motivated buying that hardware.
3. **Per-machine-class images** (one per teammate / per host). Rejected.
   High cognitive overhead, easy for dependency drift between flavors,
   nightmare for CI matrix.

## Consequences

**Costs:**

- Two Dockerfiles to keep in sync. The shared apt + pip layers must be
  updated in both places when we add a dependency. Both files cross-reference
  each other in comments to make drift visible during review.
- The build / test matrix grows. CI will need to exercise at least the CPU
  container on x86_64; the GPU container is large and is more reasonably
  built on the GPU machines themselves rather than in cloud CI.
- `fortis-dev-gpu` is multi-GB (estimated 8 to 12 GB layered). First pull on
  a typical home connection takes 10-20 minutes. Documented in
  `docker/README.md`.

**Benefits:**

- Every teammate can develop FORTIS from day one with a single `docker
  compose up`, no NVIDIA hardware required.
- GPU-equipped machines get full Isaac ROS perception out of the box, with
  no host-side ROS install drift.
- The Jetson and the FORTIS PC use the *same* image (modulo arch), so
  what works on the desk works on the robot.
- Source tree is shared between containers via the `/workspace` mount, so
  swapping containers is purely a matter of which `docker compose` command
  you ran. No code duplication.

**Pinning policy:**

- Isaac ROS line is pinned to `release-3.2`. We will not chase 4.x until the
  rest of the FORTIS stack moves off Humble.
- CPU base is pinned to `osrf/ros:humble-desktop` (the Humble LTS line).
- Both pins are reviewed at each FORTIS milestone.
