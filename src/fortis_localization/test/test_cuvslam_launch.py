"""
Static checks for the cuVSLAM scaffold launch file.

The plugin only exists in the GPU image, so no node is started here: the
launch file must parse, name the release-3.2 plugin, run it in a
multithreaded container, keep both of the plugin's TF publishers off, and
remap its odometry onto the /fortis/vo topic config/ekf_vio.yaml fuses.
"""

from __future__ import annotations

import ast
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PKG_ROOT / "launch" / "cuvslam.launch.py"


def test_cuvslam_launch_parses_and_remaps_to_fortis_vo():
    """Parse the scaffold and check its plugin, container and remaps."""
    source = LAUNCH_FILE.read_text()
    ast.parse(source)
    assert "nvidia::isaac_ros::visual_slam::VisualSlamNode" in source
    assert "component_container_mt" in source
    assert '("visual_slam/tracking/odometry", VO_TOPIC)' in source
    assert 'VO_TOPIC = "/fortis/vo"' in source
    assert '"publish_odom_to_base_tf": False' in source
    assert '"publish_map_to_odom_tf": False' in source
