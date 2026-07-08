"""
launch_testing acceptance test for the FORTIS perception chain.

Brings up the WS-I demo composition -- the synthetic OAK replayer plus
the real perception nodes and the mission FSM -- in one process and
exercises the sprint's acceptance sequence end to end:

1. the per-camera cloud publishes with a real point set,
2. the fused cloud publishes,
3. the voxel map cloud publishes with nonzero width,
4. blob detection finds the scene's red sphere and latches
   /fortis/context/grasp_candidate_ok True,
5. START_ORBIT gates the FSM from IDLE into ORBIT,
6. a /clicked_point becomes /fortis/target_pose + target_pose_valid and
   target_selector's chassis_cam_click event drives ORBIT -> TARGETING,
7. voxel_map's ~/save_map persists the map to a temp file.

Topic and service names are imported from the production modules where
they exist as constants, so a rename fails loudly here. Latched topics
(mission state, context flags, map cloud, target pose) are read with
fortis_comms' latched profile; a QoS drift between publisher and
subscriber is exactly the failure mode integration tests exist to catch.

The test methods are ORDER-DEPENDENT by design: the pipeline warms up
once and each numbered method builds on the state the previous one
established (unittest runs them in name order). Every wait is bounded;
a green run completes in well under two minutes.

Lives in fortis_integration_tests (fixed ROS_DOMAIN_ID 94 via
test/conftest.py) so the launched nodes cannot cross-talk with the
per-package unit tests.
"""

from __future__ import annotations

import os
import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from fortis_comms.qos_profiles import latched_qos_profile
from fortis_msgs.srv import SaveMap
from fortis_perception.cloud_fusion_node import FUSED_CLOUD_TOPIC
from fortis_perception.detection_node import DETECTIONS_TOPIC, GRASP_OK_TOPIC
from fortis_perception.target_selector_node import (
    CLICKED_POINT_TOPIC,
    TARGET_POSE_TOPIC,
    TARGET_VALID_TOPIC,
)
from fortis_perception.voxel_map_node import MAP_CLOUD_TOPIC
from fortis_perception.voxel_map_node import NODE_NAME as VOXEL_MAP_NODE_NAME
from fortis_safety.mission_state_machine import Event, State
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Empty, String
from vision_msgs.msg import Detection2DArray


# --- Constants ---------------------------------------------------------------

#: The one camera the replayer stands in for; all per-camera topics hang
#: off this name.
CAMERA = "oak_chassis_front"
POINTS_TOPIC = f"/fortis/perception/{CAMERA}/points"

#: mission_state_node exports no topic constant; this matches its
#: publisher and the latched contract locked in bringup_params.yaml.
MISSION_STATE_TOPIC = "/fortis/mission_state"
START_ORBIT_TOPIC = f"/fortis/events/{Event.START_ORBIT.name.lower()}"
SAVE_MAP_SERVICE = f"/{VOXEL_MAP_NODE_NAME}/save_map"

#: Class name HsvBlobDetector assigns the synthetic scene's red sphere
#: (a band key inside the detector, not an exported constant).
RED_SPHERE_CLASS = "red_sphere"

#: Click 1.5 m ahead of base_link: inside target_selector's default
#: [0.3, 3.5] m validity annulus.
CLICK_POINT_XYZ = (1.5, 0.0, 0.0)

#: A decimated 320x200 frame yields up to ~4000 points; anything above
#: this floor proves real geometry, not a degenerate empty cloud.
MIN_CLOUD_POINTS = 100

#: Per-spin_once timeout; small enough to drain promptly.
SPIN_ONCE_TIMEOUT_S: float = 0.02
#: Budget for the launched nodes to come up and latch the first state.
STARTUP_TIMEOUT_S: float = 10.0
#: First cloud is the slowest hop: camera up + camera_info + rgb/depth sync.
FIRST_CLOUD_TIMEOUT_S: float = 20.0
FUSED_TIMEOUT_S: float = 10.0
#: Map needs min_hits (3) integrations plus the 1 Hz publish timer.
MAP_TIMEOUT_S: float = 20.0
#: The sphere is visible over most of the ~21 s orbit; one period + margin.
DETECTION_TIMEOUT_S: float = 25.0
GRASP_OK_TIMEOUT_S: float = 5.0
ORBIT_TIMEOUT_S: float = 10.0
TARGETING_TIMEOUT_S: float = 15.0
SERVICE_READY_TIMEOUT_S: float = 5.0
SAVE_MAP_TIMEOUT_S: float = 10.0


@pytest.mark.launch_test
def generate_test_description():
    """Launch the perception chain + mission FSM against the replayer.

    Uses the canonical entry-point names registered by each package's
    setup.py so this test exercises the same executables a deployment
    launches. 320x200 @ 5 fps keeps the raycaster cheap on CI while
    preserving the full topic contract.
    """
    replayer = launch_ros.actions.Node(
        package='fortis_sim_support',
        executable='oak_replayer_node',
        name='oak_replayer',
        output='screen',
        parameters=[{
            'camera_name': CAMERA,
            'width': 320,
            'height': 200,
            'fps': 5.0,
            'trajectory': 'orbit',
            'scene': 'baseline',
            'publish_tf': True,
            'seed': 0,
        }],
    )
    depth_to_cloud = launch_ros.actions.Node(
        package='fortis_perception',
        executable='depth_to_cloud_node',
        name='depth_to_cloud',
        output='screen',
        parameters=[{'camera_name': CAMERA}],
    )
    cloud_fusion = launch_ros.actions.Node(
        package='fortis_perception',
        executable='cloud_fusion_node',
        name='cloud_fusion',
        output='screen',
        parameters=[{'input_topics': [POINTS_TOPIC]}],
    )
    voxel_map = launch_ros.actions.Node(
        package='fortis_perception',
        executable='voxel_map_node',
        name='voxel_map',
        output='screen',
    )
    detection = launch_ros.actions.Node(
        package='fortis_perception',
        executable='detection_node',
        name='detection',
        output='screen',
        parameters=[{'camera_name': CAMERA, 'detector': 'blob'}],
    )
    target_selector = launch_ros.actions.Node(
        package='fortis_perception',
        executable='target_selector_node',
        name='target_selector',
        output='screen',
    )
    mission_state = launch_ros.actions.Node(
        package='fortis_safety',
        executable='mission_state_node',
        name='mission_state_node',
        output='screen',
    )
    return (
        launch.LaunchDescription([
            replayer,
            depth_to_cloud,
            cloud_fusion,
            voxel_map,
            detection,
            target_selector,
            mission_state,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestPerceptionChain(unittest.TestCase):
    """Ordered end-to-end assertions; each test builds on the previous one.

    No per-test FSM reset on purpose: the numbered sequence IS the
    scenario (pipeline warm-up -> detection -> ORBIT -> TARGETING ->
    save), and unittest's name-ordered execution encodes it.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('perception_chain_test')
        latched = latched_qos_profile()

        cls.camera_clouds: list[PointCloud2] = []
        cls.fused_clouds: list[PointCloud2] = []
        cls.map_clouds: list[PointCloud2] = []
        cls.detections: list[Detection2DArray] = []
        cls.grasp_ok_msgs: list[Bool] = []
        cls.valid_msgs: list[Bool] = []
        cls.pose_msgs: list[PoseStamped] = []
        cls.state_msgs: list[String] = []

        # Sensor-style streams: best-effort subscriber against the nodes'
        # reliable publishers (compatible; matches production consumers).
        cls.node.create_subscription(
            PointCloud2, POINTS_TOPIC,
            cls.camera_clouds.append, qos_profile_sensor_data)
        cls.node.create_subscription(
            PointCloud2, FUSED_CLOUD_TOPIC,
            cls.fused_clouds.append, qos_profile_sensor_data)
        # Latched streams need the TRANSIENT_LOCAL profile or DDS silently
        # refuses the match.
        cls.node.create_subscription(
            PointCloud2, MAP_CLOUD_TOPIC, cls.map_clouds.append, latched)
        cls.node.create_subscription(
            Detection2DArray, DETECTIONS_TOPIC, cls.detections.append, 10)
        cls.node.create_subscription(
            Bool, GRASP_OK_TOPIC, cls.grasp_ok_msgs.append, latched)
        cls.node.create_subscription(
            Bool, TARGET_VALID_TOPIC, cls.valid_msgs.append, latched)
        cls.node.create_subscription(
            PoseStamped, TARGET_POSE_TOPIC, cls.pose_msgs.append, latched)
        cls.node.create_subscription(
            String, MISSION_STATE_TOPIC, cls.state_msgs.append, latched)

        cls.start_orbit_pub = cls.node.create_publisher(
            Empty, START_ORBIT_TOPIC, 10)
        cls.click_pub = cls.node.create_publisher(
            PointStamped, CLICKED_POINT_TOPIC, 10)
        cls.save_map_client = cls.node.create_client(SaveMap, SAVE_MAP_SERVICE)

        cls._wait_until(
            lambda: cls.state_msgs, STARTUP_TIMEOUT_S,
            "mission_state never published; is mission_state_node up?")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # --- Spin / wait helpers ----------------------------------------------

    @classmethod
    def _spin_for(cls, duration_s: float) -> None:
        """Spin the helper node for a fixed wall-clock duration."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    @classmethod
    def _wait_until(cls, predicate, timeout_s: float, message: str) -> None:
        """Spin until predicate() is truthy or fail with message."""
        end = time.monotonic() + timeout_s
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        assert predicate(), message

    def _current_state(self) -> str | None:
        """Return the latest observed mission state name."""
        return self.state_msgs[-1].data if self.state_msgs else None

    def _click_msg(self) -> PointStamped:
        """Build a click 1.5 m from base_link, inside the valid annulus."""
        msg = PointStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = CLICK_POINT_XYZ
        return msg

    # --- Tests --------------------------------------------------------------

    def test_01_camera_cloud_publishes(self):
        """Verify the per-camera cloud arrives with a real point set."""
        self._wait_until(
            lambda: any(c.width * c.height > MIN_CLOUD_POINTS
                        for c in self.camera_clouds),
            FIRST_CLOUD_TIMEOUT_S,
            f"no cloud with > {MIN_CLOUD_POINTS} points arrived on "
            f"{POINTS_TOPIC} within {FIRST_CLOUD_TIMEOUT_S}s; is the "
            "replayer -> depth_to_cloud seam broken?",
        )

    def test_02_fused_cloud_publishes(self):
        """Verify cloud_fusion publishes a non-empty base_link cloud."""
        self._wait_until(
            lambda: any(c.width * c.height > 0 for c in self.fused_clouds),
            FUSED_TIMEOUT_S,
            f"no fused cloud arrived on {FUSED_CLOUD_TOPIC} within "
            f"{FUSED_TIMEOUT_S}s; is the base_link <- optical TF flowing?",
        )

    def test_03_map_cloud_nonzero(self):
        """Verify the voxel map accumulates occupied voxels."""
        self._wait_until(
            lambda: any(c.width > 0 for c in self.map_clouds),
            MAP_TIMEOUT_S,
            f"voxel_map never published a non-empty cloud on "
            f"{MAP_CLOUD_TOPIC} within {MAP_TIMEOUT_S}s; is the "
            "odom <- base_link TF flowing?",
        )

    def test_04_red_sphere_detected_and_grasp_ok(self):
        """Verify blob detection finds the red sphere and latches grasp ok."""
        def red_sphere_seen() -> bool:
            return any(
                hyp.hypothesis.class_id == RED_SPHERE_CLASS
                for arr in self.detections
                for det in arr.detections
                for hyp in det.results
            )
        self._wait_until(
            red_sphere_seen, DETECTION_TIMEOUT_S,
            f"no {RED_SPHERE_CLASS!r} detection on {DETECTIONS_TOPIC} "
            f"within {DETECTION_TIMEOUT_S}s; the baseline scene contains "
            "one red sphere the blob detector must find",
        )
        # The same detection pass that saw the sphere publishes the grasp
        # candidate; only DDS delivery separates the two.
        self._wait_until(
            lambda: self.grasp_ok_msgs and self.grasp_ok_msgs[-1].data,
            GRASP_OK_TIMEOUT_S,
            f"{GRASP_OK_TOPIC} never latched True after a red_sphere "
            "detection; is the graspable-class / range gate broken?",
        )

    def test_05_start_orbit_gates_fsm_to_orbit(self):
        """Publish START_ORBIT and verify the FSM leaves IDLE for ORBIT."""
        deadline = time.monotonic() + ORBIT_TIMEOUT_S
        while (self._current_state() != State.ORBIT.name
               and time.monotonic() < deadline):
            # Re-publish: the first event can race DDS subscription
            # matching; a duplicate in ORBIT is rejected noise (try_step).
            self.start_orbit_pub.publish(Empty())
            self._spin_for(0.5)
        self.assertEqual(
            self._current_state(), State.ORBIT.name,
            f"FSM did not reach ORBIT within {ORBIT_TIMEOUT_S}s "
            f"(last seen: {self._current_state()!r})",
        )

    def test_06_click_produces_target_and_targeting(self):
        """Verify a valid click yields pose + flag and ORBIT -> TARGETING."""
        self.assertEqual(self._current_state(), State.ORBIT.name)
        deadline = time.monotonic() + TARGETING_TIMEOUT_S
        while (self._current_state() != State.TARGETING.name
               and time.monotonic() < deadline):
            # Re-click until the transition lands: target_selector's
            # context flag and chassis_cam_click event travel on separate
            # topics, so the FSM can see the event before the guard flag.
            self.click_pub.publish(self._click_msg())
            self._spin_for(1.0)
        self.assertEqual(
            self._current_state(), State.TARGETING.name,
            f"FSM did not reach TARGETING within {TARGETING_TIMEOUT_S}s "
            f"(last seen: {self._current_state()!r})",
        )
        self.assertTrue(
            self.pose_msgs,
            f"no target pose arrived on {TARGET_POSE_TOPIC} for an "
            "in-annulus click",
        )
        self.assertTrue(
            self.valid_msgs and self.valid_msgs[-1].data,
            f"{TARGET_VALID_TOPIC} should be latched True after an "
            "accepted click",
        )

    def test_07_save_map_writes_file(self):
        """Call save_map with a temp path and verify success + file on disk."""
        self._wait_until(
            self.save_map_client.service_is_ready, SERVICE_READY_TIMEOUT_S,
            f"{SAVE_MAP_SERVICE} never became available",
        )
        tmp_dir = tempfile.mkdtemp(prefix='fortis_chain_')
        path = os.path.join(tmp_dir, 'map.npz')
        request = SaveMap.Request()
        request.path = path
        future = self.save_map_client.call_async(request)
        deadline = time.monotonic() + SAVE_MAP_TIMEOUT_S
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        self.assertTrue(
            future.done(),
            f"save_map did not respond within {SAVE_MAP_TIMEOUT_S}s",
        )
        response = future.result()
        self.assertTrue(response.success, f"save_map failed: {response.message}")
        self.assertTrue(
            os.path.isfile(path),
            f"save_map reported success but {path} does not exist",
        )
        os.remove(path)
        os.rmdir(tmp_dir)
