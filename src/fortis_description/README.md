# fortis_description

Robot description package for FORTIS: URDF / xacro source, meshes, RViz config, and description-only launch files.

**Currently a scaffold.** The first OnShape URDF export has landed but requires cleanup before integration -- the export came across with **95 links / 94 joints**, with naming and topology issues Adrian + Carlos are working through against a fix list. URDF authoring is planned as a **dual track**: the chassis description comes from the OnShape cleanup, the 4-DOF arm is hand-authored in xacro so the joint frames line up with `fortis_arm` and the Teensy firmware contract without having to fight the CAD export. This package exists so other packages (`fortis_moveit_config`, `fortis_localization`, anything that needs `robot_description`) can declare a dependency on it without the dependency being a forward reference.

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

- [x] Receive first OnShape URDF + mesh export (Adrian + Carlos).
- [ ] Clean up the OnShape export (95 links / 94 joints, naming + topology issues) per Adrian + Carlos's fix list before merging into `urdf/`.
- [ ] Land `urdf/fortis.urdf.xacro` plus per-subassembly macros. Dual-track: chassis pulled from the cleaned-up OnShape export, 4-DOF arm hand-authored in xacro.
- [ ] Land `meshes/visual/` and `meshes/collision/` mirroring the URDF link tree.
- [ ] Add `launch/display.launch.py` (robot_state_publisher + joint_state_publisher_gui + RViz).
- [ ] Add `rviz/fortis.rviz` with the default RobotModel + TF + Camera panels for OAK-D Lite quad layout.
- [ ] Add SRDF + XRDF in `fortis_moveit_config` once URDF is stable.

## Frame names

Locked. See `CLAUDE.md` "Frame naming conventions" — joint names are referenced by `fortis_safety`, `fortis_drive`, and the integration tests; renaming requires a cross-package commit.
