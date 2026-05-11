"""Emit a default RViz2 .rviz config that loads the FORTIS chassis URDF.

A minimal config that opens with the RobotModel + TF + Grid panels
visible so an operator can verify the URDF after a tool run without
hand-authoring an RViz layout. We do not try to mirror the production
operator UI; that lives in ``fortis_description/rviz/`` and is owned
by the bringup work.

The config is a small JSON-ish YAML blob -- not a full RViz layout
dump, just enough panels + displays to get the robot on screen.
"""

from __future__ import annotations

from pathlib import Path
from textwrap import dedent


RVIZ_TEMPLATE = """\
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Plane: XY
      Color: 160; 160; 160
      Cell Size: 0.5
      Plane Cell Count: 20
      Enabled: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: true
      Show Axes: true
    - Class: rviz_default_plugins/RobotModel
      Description Source: Topic
      Description Topic:
        Value: /robot_description
        Depth: 1
        Durability Policy: Transient Local
        Reliability Policy: Reliable
      Enabled: true
      Name: RobotModel
      Visual Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.0
      Focal Point:
        X: 0.0
        Y: 0.0
        Z: 0.0
      Pitch: 0.5
      Yaw: 0.5
      Name: Current View
Window Geometry:
  Height: 800
  Width: 1200
"""


def generate_rviz_config(
    output_path: Path,
    fixed_frame: str = "base_link",
) -> Path:
    """Write a minimal RViz config that loads the cleaned URDF on bringup.

    Returns the path the config was written to. The default fixed frame
    is ``base_link``; override if the cleaned URDF uses a different
    name for the chassis link.
    """
    body = RVIZ_TEMPLATE
    if fixed_frame != "base_link":
        body = body.replace("Fixed Frame: base_link", f"Fixed Frame: {fixed_frame}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(body, encoding="utf-8")
    return output_path
