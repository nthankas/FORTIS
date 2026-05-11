# FORTIS Docker containers

FORTIS ships two development containers. Both mount the repo at `/workspace`,
both build the same ROS 2 Humble source tree, and both run `colcon build` /
`colcon test` the same way. The only difference is whether NVIDIA-accelerated
Isaac ROS packages are available inside the container.

## Which container should I use?

| Container         | When to use it                                                                                          | Hardware required          | Isaac ROS |
|-------------------|---------------------------------------------------------------------------------------------------------|----------------------------|-----------|
| `fortis-dev`      | Default for everyone. Works on any x86_64 or aarch64 Docker host. CPU-only build of the FORTIS stack.   | None (any Linux/Mac/WSL2)  | No        |
| `fortis-dev-gpu`  | FORTIS PC (RTX 4080), IdeaPad (RTX 5050 Laptop), Jetson Orin Nano Super. Adds GPU-accelerated perception. | NVIDIA GPU + Container Toolkit | Yes (3.2) |

If you do not have an NVIDIA GPU, use `fortis-dev`. The full FORTIS source
tree builds in either container; nodes that import Isaac ROS packages will
not build (or will be skipped) in `fortis-dev`, and that is expected.

## Prerequisites

Common (both containers):
- Docker 24+ with the Compose plugin (`docker compose`, not `docker-compose`).
- This repository checked out somewhere on your host.

`fortis-dev-gpu` only:
- Recent NVIDIA driver (R535+ on x86_64; JetPack 6.x on Jetson).
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
  installed and configured: `sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker`.
- Verify with `docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi`.
- **NGC login.** The Isaac ROS Dev Base image lives on `nvcr.io` and
  requires an account even though it is nominally public. Before the
  first GPU build, run `docker login nvcr.io` and paste your NGC API
  key (free account at `ngc.nvidia.com`). Without this the build will
  fail at the `FROM` stage with `unauthorized`.

## Build

### CPU container (`fortis-dev`)

```bash
docker compose -f docker/docker-compose.yml build
```

### GPU container (`fortis-dev-gpu`)

The GPU compose file is an *overlay* on top of the base compose file. Always
include both with `-f`:

```bash
docker compose -f docker/docker-compose.yml \
               -f docker/docker-compose.gpu.yml build
```

First pull is slow: the Isaac ROS Dev Base image is multi-GB (roughly 8 to
12 GB once layered) so expect 10-20 minutes on a typical home connection.

## Run

### CPU container

```bash
docker compose -f docker/docker-compose.yml up -d
docker exec -it fortis-dev /bin/bash
```

### GPU container

```bash
docker compose -f docker/docker-compose.yml \
               -f docker/docker-compose.gpu.yml up -d
docker exec -it fortis-dev-gpu /bin/bash
```

Inside either container the workspace lives at `/workspace`. To build and
test:

```bash
cd /workspace
colcon build --symlink-install
colcon test
```

## What's different between the two

- `fortis-dev` is `FROM osrf/ros:humble-desktop` (CPU only, ubuntu 22.04).
- `fortis-dev-gpu` is `FROM nvcr.io/nvidia/isaac/ros:<hash>-<arch>` where
  `<hash>` is a content-addressed Isaac ROS build (pinned in
  `Dockerfile.dev-gpu` via the `ISAAC_ROS_HASH` ARG) and `<arch>` is
  `amd64` or `arm64-jetpack`. The Isaac ROS apt repository is
  pre-configured and these packages pre-installed:
  - `ros-humble-isaac-ros-visual-slam`
  - `ros-humble-isaac-ros-nvblox`
  - `ros-humble-isaac-ros-image-proc`
  - `ros-humble-isaac-ros-image-pipeline`
  - `ros-humble-isaac-ros-cumotion`
  - `ros-humble-isaac-ros-cumotion-moveit`

Both containers install the same `ros-humble-moveit`, `ros2_control`,
`robot_localization`, `foxglove_bridge`, `xacro` packages plus the same pip
deps (`depthai`, `python-can`, `pyserial`, `odrive`, numpy/scipy/matplotlib),
and both create the same `FORTIS` user with passwordless sudo.

## Architecture notes (multi-arch)

Both Dockerfiles are TARGETARCH-aware:
- `fortis-dev` works on x86_64 and aarch64 hosts because `osrf/ros:humble-desktop`
  is multi-arch on Docker Hub.
- `fortis-dev-gpu` selects between the `-amd64` and `-arm64-jetpack` Isaac
  ROS Dev Base tags at build time. Jetson developers get
  `-arm64-jetpack` automatically (BuildKit picks it when building on the
  Jetson itself; on x86_64 you can request it via `--platform linux/arm64`
  plus QEMU, though native builds on-Jetson are faster). NVIDIA also
  publishes `-arm64-fastos`; FORTIS does not use it because that variant
  targets NVIDIA's AMR appliances, not JetPack-based Jetson devkits.

## Re-pinning the Isaac ROS base image

The Isaac ROS Dev Base on NGC is hash-tagged, not semver-tagged. The
current pin and re-pin procedure live in the header comment of
`docker/Dockerfile.dev-gpu`. In short: re-pin when an apt install or
an Isaac ROS package starts failing inside `fortis-dev-gpu`, or as a
quarterly hygiene tick. NGC garbage-collects old hashes on its own
schedule, so a stale pin will eventually 404 at build time.

## Troubleshooting

- "docker: Error response from daemon: could not select device driver" -> the
  NVIDIA Container Toolkit is not installed or not configured. See prereqs.
- Isaac ROS apt package missing -> NVIDIA occasionally renames or temporarily
  drops binaries. Run `apt-cache search ros-humble-isaac-ros` inside the
  container; the Dockerfile install layer is non-fatal so the image still
  builds even if a single package is unavailable.
- Display does not appear -> ensure `xhost +local:` on the host before
  starting the container.

For the rationale behind the dual-container approach see
[`docs/adr/0001-dual-container-strategy.md`](../docs/adr/0001-dual-container-strategy.md).
