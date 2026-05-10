# fortis_description

Robot description package for FORTIS: URDF / xacro source, meshes, RViz config, and description-only launch files.

**Currently an empty scaffold.** The URDF is blocked on the OnShape export (Adrian + Carlos own that work). This package exists so other packages (`fortis_moveit_config`, `fortis_localization`, anything that needs `robot_description`) can declare a dependency on it without the dependency being a forward reference.

## Layout

```
fortis_description/
  urdf/        TODO -- xacro source, see urdf/TODO.md
  meshes/      TODO -- visual + collision meshes, see meshes/PLACEHOLDER.md
  launch/      TODO -- display.launch.py for robot_state_publisher + RViz
  config/      TODO -- joint_state_publisher_gui defaults, etc.
  rviz/        TODO -- rviz config files (fortis.rviz)
```

## Building

```bash
colcon build --packages-select fortis_description
source install/setup.bash
```

The build is currently a no-op installer for the empty asset directories. It establishes the package in the ament index so dependents can resolve it.

## TODO

- [ ] Receive OnShape URDF + mesh export (Adrian + Carlos).
- [ ] Land `urdf/fortis.urdf.xacro` plus per-subassembly macros.
- [ ] Land `meshes/visual/` and `meshes/collision/` mirroring the URDF link tree.
- [ ] Add `launch/display.launch.py` (robot_state_publisher + joint_state_publisher_gui + RViz).
- [ ] Add `rviz/fortis.rviz` with the default RobotModel + TF + Camera panels for OAK-D Lite quad layout.
- [ ] Add SRDF + XRDF in `fortis_moveit_config` once URDF is stable.

## Frame names

Locked. See `CLAUDE.md` "Frame naming conventions" — joint names are referenced by `fortis_safety`, `fortis_drive`, and the integration tests; renaming requires a cross-package commit.
