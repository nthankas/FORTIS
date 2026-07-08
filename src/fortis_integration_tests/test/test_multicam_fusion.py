"""
launch_testing acceptance test for 4-camera synthetic cloud fusion.

Brings up the full synthetic rig -- one oak_replayer per chassis mount
(front/rear/left/right), one depth_to_cloud per camera, and
cloud_fusion subscribed to all four cloud topics (the same composition
perception.launch.py synthetic:=true assembles) -- and asserts fusion
actually merges the four views: /fortis/perception/points_fused
arrives and carries MORE points than any single camera's cloud.

Only the front replayer owns odom->base_link + ground truth
(publish_base_tf); the other three broadcast just their static mount
TFs, which is all cloud_fusion needs to bring their optical-frame
clouds into base_link. cloud_fusion runs with voxel_size 0.0 so the
fused count is the exact concatenation of its inputs; with
downsampling on, voxel merging could legitimately shrink the fused
cloud below one camera's raw count and the merge assertion would be
meaningless.

Hold trajectory: the robot stares from (1, 0) at four disjoint parts
of the walled baseline scene (pillar side, +X wall, +/-Y walls), so
every camera returns a nonzero cloud within depth_to_cloud's 6 m
range. Lives in fortis_integration_tests (fixed ROS_DOMAIN_ID 94 via
test/conftest.py) so the rig cannot cross-talk with per-package tests.
"""

from __future__ import annotations

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from fortis_perception.cloud_fusion_node import FUSED_CLOUD_TOPIC
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


# --- Constants ---------------------------------------------------------------

#: All four chassis mounts, front first: it owns publish_base_tf.
MOUNTS = ("front", "rear", "left", "right")
CAMERAS = tuple(f"oak_chassis_{mount}" for mount in MOUNTS)
POINTS_TOPICS = {
    camera: f"/fortis/perception/{camera}/points" for camera in CAMERAS
}

#: Per-spin_once timeout; small enough to drain promptly.
SPIN_ONCE_TIMEOUT_S: float = 0.02
#: Four raycasters render concurrently; the perception-chain first-cloud
#: budget (20 s) with headroom for CPU contention on CI.
ALL_CLOUDS_TIMEOUT_S: float = 60.0
FUSED_TIMEOUT_S: float = 20.0


@pytest.mark.launch_test
def generate_test_description():
    """Launch the 4-camera synthetic rig, per-camera clouds, and fusion.

    320x200 @ 5 fps keeps four concurrent raycasters cheap on CI while
    preserving the full topic contract of the real rig.
    """
    replayers = [
        launch_ros.actions.Node(
            package='fortis_sim_support',
            executable='oak_replayer_node',
            name=f'oak_replayer_{mount}',
            output='screen',
            parameters=[{
                'mount': mount,
                'camera_name': camera,
                'width': 320,
                'height': 200,
                'fps': 5.0,
                'trajectory': 'hold',
                'scene': 'baseline',
                'publish_tf': True,
                'publish_base_tf': mount == 'front',
                'seed': 0,
            }],
        )
        for mount, camera in zip(MOUNTS, CAMERAS)
    ]
    clouds = [
        launch_ros.actions.Node(
            package='fortis_perception',
            executable='depth_to_cloud_node',
            name=f'depth_to_cloud_{camera}',
            output='screen',
            parameters=[{'camera_name': camera}],
        )
        for camera in CAMERAS
    ]
    cloud_fusion = launch_ros.actions.Node(
        package='fortis_perception',
        executable='cloud_fusion_node',
        name='cloud_fusion',
        output='screen',
        parameters=[{
            'input_topics': [POINTS_TOPICS[camera] for camera in CAMERAS],
            # No downsampling: the fused count must be the exact
            # concatenation for the merge assertion to mean anything.
            'voxel_size': 0.0,
        }],
    )
    return (
        launch.LaunchDescription(
            replayers + clouds
            + [cloud_fusion, launch_testing.actions.ReadyToTest()]),
        {},
    )


class TestMulticamFusion(unittest.TestCase):
    """Ordered assertions: per-camera clouds first, then the fused merge."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('multicam_fusion_test')
        cls.camera_counts: dict[str, list[int]] = {c: [] for c in CAMERAS}
        cls.fused_counts: list[int] = []
        for camera in CAMERAS:
            cls.node.create_subscription(
                PointCloud2, POINTS_TOPICS[camera],
                lambda msg, name=camera: cls.camera_counts[name].append(
                    msg.width * msg.height),
                qos_profile_sensor_data)
        cls.node.create_subscription(
            PointCloud2, FUSED_CLOUD_TOPIC,
            lambda msg: cls.fused_counts.append(msg.width * msg.height),
            qos_profile_sensor_data)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _wait_until(cls, predicate, timeout_s: float, message: str) -> None:
        """Spin until predicate() is truthy or fail with message."""
        end = time.monotonic() + timeout_s
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        assert predicate(), message

    def test_01_every_camera_cloud_publishes(self):
        """Verify all four per-camera clouds arrive with points."""
        def all_cameras_nonzero() -> bool:
            return all(any(count > 0 for count in counts)
                       for counts in self.camera_counts.values())
        self._wait_until(
            all_cameras_nonzero, ALL_CLOUDS_TIMEOUT_S,
            "not every camera produced a nonzero cloud on its "
            "/fortis/perception/<cam>/points topic within "
            f"{ALL_CLOUDS_TIMEOUT_S}s; is the 4-replayer -> depth_to_cloud "
            "seam broken?",
        )

    def test_02_fused_cloud_exceeds_any_single_camera(self):
        """Verify the fused cloud merges more points than any one camera."""
        def fused_merged() -> bool:
            if not self.fused_counts:
                return False
            biggest_single = max(
                max(counts) for counts in self.camera_counts.values())
            return self.fused_counts[-1] > biggest_single
        self._wait_until(
            fused_merged, FUSED_TIMEOUT_S,
            f"fused cloud on {FUSED_CLOUD_TOPIC} never exceeded the largest "
            "single-camera cloud; is cloud_fusion merging all four inputs "
            "(static mount TFs from every replayer)?",
        )
