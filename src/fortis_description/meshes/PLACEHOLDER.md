# meshes/

Visual + collision meshes referenced by the URDF.

**Currently empty.** Blocked on OnShape export.

## What goes here

- `visual/*.dae` (or `.stl`) — high-detail Collada meshes for RViz / RViz2.
- `collision/*.stl` — simplified convex meshes for collision checking. Keep face count low; MoveIt and Nvblox both pay for high-poly collision geometry.
- `chassis/`, `arm/`, `wheels/`, `cameras/`, `gripper/` — one subdirectory per subassembly so the URDF references match the OnShape source tree.

## Conventions

- Meters, not millimeters. OnShape exports default to meters; if a mesh comes out 1000x scaled, the export was wrong, not the URDF.
- Origin at the joint frame, not the part centroid. URDF `<origin>` should rarely need anything beyond a default pose to place a mesh correctly.
- One file per link. Multi-link STLs make collision overrides impossible without re-export.

## TODO

- [ ] Receive OnShape mesh export (Adrian + Carlos).
- [ ] Generate simplified collision meshes for each visual mesh.
- [ ] Confirm meters scale matches the URDF dimensions.
