# Contributing to FORTIS

This is the working document for how this repo is organized and how to make changes to it. Read this once before opening your first PR.

## Dev environment

The supported environment is the dev container. Open the repo in VSCode and use "Dev Containers: Reopen in Container" (F1). The container is ROS 2 Humble desktop with MoveIt 2, robot_localization, ros2_control, foxglove_bridge, and xacro pre-installed. Outside VSCode you can bring it up with `docker compose -f docker/docker-compose.yml up -d` and `docker exec -it fortis-dev bash`.

Inside the container the repo is mounted at `/workspace`. ROS 2 is sourced automatically. `install/setup.bash` is sourced at shell start if it exists.

Isaac Sim runs on the Windows host, not in the dev container.

## Where things go

| Kind of code | Where | Why |
|---|---|---|
| ROS 2 packages (nodes, launch files, msg defs) | `src/<package_name>/` | Anything you want to `colcon build` and `ros2 run` |
| Pure-Python libraries (no ROS dependency) | `control/<library_name>/` | Reusable in notebooks, scripts, future nodes; no ROS coupling |
| Isaac Sim scripts | `sim/isaac/...` | Runs on the Windows host with Isaac Sim 5.1 |
| Microcontroller firmware | `firmware/` | Reserved; nothing here yet |
| Reference code from earlier design rounds | `legacy/` | Read-only. Not in the build. |
| Drivetrain / arm analysis writeups | `analysis/` | Markdown only. No code. |

If you're not sure whether your code is "pure Python" or "ROS code", ask: does it `import rclpy` or use a ROS message type? If yes, it's a ROS package and lives in `src/`.

## Conventions

- ROS 2 packages: `ament_python` for now (we're not writing C++ yet). Each package has a `package.xml`, `setup.py`, `setup.cfg`, `resource/<pkg>` marker, `<pkg>/` source dir, `test/` dir.
- Package names: `fortis_<subsystem>`. Existing: `fortis_safety`. Planned: `fortis_description`, `fortis_drive`, `fortis_arm`, `fortis_perception`, `fortis_localization`, `fortis_msgs`, `fortis_bringup`.
- Topics: `/fortis/<group>/<thing>` (e.g. `/fortis/mission_state`, `/fortis/events/start_orbit`, `/fortis/context/ik_ok`).
- Pure-Python directory names: `fortis_<thing>` for parity with the ROS packages they may eventually feed into.
- No emojis in source, no em-dashes in docs.
- Comments: explain *why*, not *what*. If a future reader couldn't reasonably guess the constraint or rationale, write it down.
- Don't add error handling for paths that can't happen. Trust internal code.

## Building and running

ROS 2 build (from `/workspace`):

```bash
colcon build --packages-select fortis_safety
source install/setup.bash
ros2 run fortis_safety mission_state_node
```

`colcon build` without `--packages-select` rebuilds everything in `src/`. Use it when you change a dependency.

Re-source `install/setup.bash` whenever you add a new package or new entry point.

## Tests

ROS 2 package tests:

```bash
cd /workspace
source install/setup.bash
python3 -m pytest src/fortis_safety/test/ -v
```

`fortis_safety` deliberately keeps the FSM logic in `mission_state_machine.py` with no ROS imports so the unit tests run without `rclpy.init()`. Future packages should follow the same split: ROS-free logic in one module, ROS wrapper in another.

Pure-Python library tests:

```bash
cd /workspace/control/fortis_comms
python3 -m pytest -v
```

The fortis_comms tests run from inside the package directory so the local imports work. They don't need ROS sourced.

## Commits and PRs

- Use small, focused commits. A reorg commit and a docs commit shouldn't be the same commit.
- Commit messages should explain the *why* in the body when it isn't obvious from the subject.
- Don't push to `main` while a build is broken.

## What's blocked on what

- `fortis_description` (URDF) is blocked on Adrian's OnShape model export. Don't try to write the URDF from scratch.
- `fortis_drive` is blocked on `fortis_description` (it needs the joint names and frames).
- `fortis_arm` is blocked on `fortis_description` for the same reason.
- Sim 2 (R0 port entry) is the next active sim; `sim/isaac/xdrive/docs/R0_ENTRY_PLAN.md` is the starting point.

## Hard rules

- Never regenerate `sim/isaac/xdrive/assets/diiid_reactor.usd`. The collision mesh was applied manually in the Isaac Sim GUI; rebuilding the file from source destroys that.
- Hardware specs in this repo (chassis dims, masses, motor selections) may lag reality. The OnShape model and the BOM are authoritative. If a number in `analysis/` or in a sim script disagrees with OnShape, OnShape wins.
- Procurement constraints from the program: no parts from Chinese vendors, no marketplace sites (Amazon, eBay, AliExpress). Stick to recognized industrial suppliers (McMaster, Misumi, AndyMark, ODrive, etc.).
