# urdf_cleanup

Host-level dev tool that turns an OnShape URDF export into a clean,
ROS 2 Humble-ready URDF + launch + RViz bundle.

This is **not** a colcon package. It lives outside `src/`, runs at
the host level on checked-out URDF artefacts, and is invoked with
plain `python -m urdf_cleanup`. The `fortis_description` colcon
package consumes the output, not the tool itself.

## Why this exists

OnShape's URDF export plug-in produces output that ROS 2 Humble
cannot consume as-is. The May 10 FORTIS export shipped:

- 95 links and 94 joints, the vast majority of them spacers and
  helper bodies (`1p5_spacer*`, `part_<N>_<M>*`,
  `open_cascade_step_translator_*`).
- Mangled link and joint names with `__<N>__` suffixes from
  OnShape's `<N>` instance markers.
- 4 wheel hubs attached to a synthetic `root` node via
  `hanging_node_to_root_joint_*` rather than to the chassis frame,
  which leaves the wheel revolute joints unreachable from
  `base_link` in the TF tree.
- A ROS 1 `launch.xml` instead of a ROS 2 Humble `launch.py`.
- Mesh filenames matching the mangled link names.

Cleaning all of this by hand on every re-export is the kind of
small, mechanical work that's worth automating.

## What it does

```
raw URDF + meshes/                     cleaned URDF + launch + rviz + meshes/
+---------------------+                +-------------------------+
| 95 links            |                | ~5 to 25 meaningful     |
| 94 joints           |                |   links                 |
| hanging wheel hubs  |  --pipeline->  | 4 wheel revolute joints |
| mangled names       |                |   parented to base_link |
| ROS 1 launch.xml    |                | clean ROS 2 launch.py   |
| no ros2_control     |                | <ros2_control> block    |
+---------------------+                | RViz display config     |
                                       | meshes/ renamed         |
                                       +-------------------------+
```

The pipeline is the union of seven modules under
`urdf_cleanup/`:

| Module | Responsibility |
|---|---|
| `parser` | wraps `lxml.etree` so the rest of the pipeline gets a `UrdfDoc` with link / joint accessors instead of raw element trees |
| `link_filter` | drops nuisance leaf links matching the patterns above plus the fixed joints attaching them. Load-bearing links are kept and reported |
| `topology_fixer` | rewires joints matching `hanging_node_to_root_joint_*` from the synthetic `root` onto the chassis frame. Drops self-loop joints. Removes the synthetic `root` once nothing references it |
| `renamer` | applies a flat mangled-name -> clean-name map across links, joints, and joint parent/child references. Names absent from the map are left untouched and reported |
| `ros2_control` | injects a `<ros2_control>` block describing the wheel revolute joints with `mock_components/GenericSystem` as the default plugin |
| `mesh_renamer` | rewrites `<mesh filename=...>` to clean basenames and copies the underlying `.stl` files into the output `meshes/` |
| `launch_generator` + `rviz_generator` | emit a ROS 2 Humble `display.launch.py` and a minimal `.rviz` that loads the cleaned URDF in RViz |

Pipeline order is fixed in `main.run()`: topology fixing runs
**before** the renamer because it matches joints by their original
(mangled) names; the chassis link is reverse-looked-up from the
mapping so the topology fixer sees the still-mangled chassis name.

## Quickstart

```bash
cd tools/urdf_cleanup
pip install -r requirements.txt
python -m urdf_cleanup \
    --input  /path/to/onshape_export/    \
    --output /path/to/cleaned/           \
    --mapping configs/fortis_chassis_mapping.yaml \
    --verbose
```

`--input` accepts either a directory containing a `.urdf` (with
`meshes/` alongside) or a single `.urdf` file. `--output` is a
directory that will be created if missing; the cleaned bundle
lands at:

```
<output>/
  urdf/<input_basename>.urdf
  launch/display.launch.py
  rviz/fortis.rviz
  meshes/<renamed>.stl
```

Drop the result into `src/fortis_description/` to consume from a
colcon build.

## Mapping config

The mapping is a YAML file describing how to rename mangled
identifiers and which link the topology fixer should treat as the
chassis. Schema:

```yaml
chassis_link: base_link

link_mapping:
  base_link__1__: base_link
  wheel_hub_fl__1__: front_left_wheel_link
  ...

joint_mapping:
  hanging_node_to_root_joint_1: front_left_wheel_joint
  ...
```

A FORTIS default lives at `configs/fortis_chassis_mapping.yaml`.
Refresh it whenever Adrian + Carlos re-export -- the `__<N>__`
suffixes change with every OnShape re-version of the assembly.

## CLI flags

| Flag | Default | Purpose |
|---|---|---|
| `--input` (required) | -- | input `.urdf` file or directory |
| `--output` (required) | -- | output directory (created if missing) |
| `--mapping` | `configs/fortis_chassis_mapping.yaml` | mapping YAML; pass `/dev/null` (or any non-existent path) to skip renaming entirely |
| `--verbose` | off | one log line per pipeline step plus an unmapped-names dump |

Exit codes: `0` success, `2` for any uncaught exception during the
pipeline. The CLI prints the exception message to stderr before
exiting.

## Tests

Run from the tool root:

```bash
cd tools/urdf_cleanup
pytest -v
```

There are two fixture URDFs under `tests/fixtures/`:

- `minimal_export.urdf` - the smallest URDF that exercises every
  pipeline step. 6 links, 5 joints.
- `realistic_export.urdf` - mirrors the patterns the real May 10
  FORTIS OnShape export shipped. 15 links, 14 joints. The
  end-to-end test confirms this lands at 5 / 4 (chassis + 4
  wheels) after the pipeline.

The tests do **not** run inside colcon; `colcon test` ignores
this directory entirely because the tool sits outside `src/`.

## Limitations

- The tool handles **naming and topology**. CAD-level URDF errors
  (wrong joint origins, missing inertias, bogus masses) are out
  of scope; OnShape is the source of truth there.
- Self-loop drops trigger only for `hanging_node_to_root_joint_*`
  pattern hits with `child == chassis`. A custom mapping with a
  different hanging-joint pattern needs its own constructor args.
- The arm URDF is **not** in scope; FORTIS authors the 4-DOF arm
  by hand in xacro per the project's dual-track URDF plan
  (chassis from OnShape cleanup, arm hand-authored).
- The default `<ros2_control>` plugin is
  `mock_components/GenericSystem`. Swap to
  `odrive_ros2_control/...` (or whatever real plugin name lands)
  by editing the cleaned URDF, or pass an explicit plugin name
  to `Ros2ControlInjector` when calling the pipeline from Python.
