"""
launch_testing validation for the fortis_bringup composition.

Includes bringup.launch.py (the real, deployed launch file) under the
launch_testing harness and asserts two minimum-viable properties:

1. Every node that bringup.launch.py is supposed to compose is alive
   shortly after launch. Catches silent regressions where a refactor
   drops a Node from the launch graph.
2. The composed system actually latches /fortis/mission_state to IDLE
   within a small startup budget. Catches the failure mode where every
   node comes up but mission_state_node never publishes (wrong QoS,
   wrong topic name, dead-locked startup).

The test uses IncludeLaunchDescription against the installed share
directory of fortis_bringup rather than constructing its own Node
actions, so a divergence between the test's launch graph and the
deployed one is impossible by construction.

Lives in fortis_integration_tests for the same reason as
test_safety_drive_integration: pytest collects a package's test
directory as one process, and the launched nodes here would cross-talk
with unit tests in fortis_drive/fortis_safety if they shared a
process. The package's conftest also pins ROS_DOMAIN_ID per PID so
this suite is isolated from sibling test packages running in
parallel under colcon test.
"""

from __future__ import annotations

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from fortis_safety.mission_state_machine import State
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


# --- Constants --------------------------------------------------------------

#: Node names that bringup.launch.py is contractually expected to
#: compose. If a future include drops or renames one, this list must
#: change in the same commit so the test failure is descriptive.
EXPECTED_NODE_NAMES: tuple[str, ...] = (
    "mission_state_node",
    "drive_node",
)

#: Per-spin_once timeout. Short enough to drain the executor promptly,
#: long enough to avoid burning CPU when nothing is queued.
SPIN_ONCE_TIMEOUT_S: float = 0.05

#: Hard ceiling on waiting for both expected nodes to appear in
#: get_node_names(). DDS discovery on a cold start typically takes
#: 1-3 s; 10 s leaves headroom for a loaded CI box without making a
#: real launch failure look like a slow start.
NODE_DISCOVERY_TIMEOUT_S: float = 10.0

#: Hard ceiling on waiting for the latched mission_state message.
#: mission_state_node publishes IDLE immediately on construction with
#: TRANSIENT_LOCAL durability, so a 5 s budget covers process start +
#: DDS match + delivery on a slow box.
STATE_DELIVERY_TIMEOUT_S: float = 5.0

#: Mission state expected on startup. Matches the initial state of
#: MissionStateMachine and the first message mission_state_node
#: publishes from its constructor.
EXPECTED_STARTUP_STATE: str = State.IDLE.name


# --- Launch description -----------------------------------------------------


@pytest.mark.launch_test
def generate_test_description():
    """
    Launch description: include the real fortis_bringup composition.

    Resolves bringup.launch.py from the installed share directory
    rather than the source tree so the test exercises whatever the
    build system actually shipped. ReadyToTest signals launch_testing
    that fixtures are up and the test methods can begin.
    """
    bringup_share = get_package_share_directory("fortis_bringup")
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [bringup_share, "/launch/bringup.launch.py"]
        )
    )
    return (
        launch.LaunchDescription([
            bringup_launch,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


# --- Test class -------------------------------------------------------------


class TestBringupLaunch(unittest.TestCase):
    """Launch-graph validation for fortis_bringup composition."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("bringup_launch_test")

        # Matched to the publisher in fortis_safety/mission_state_node:
        # TRANSIENT_LOCAL + RELIABLE depth=1. A mismatched profile
        # causes DDS to silently refuse the match, which is exactly
        # the failure mode this integration test must catch.
        cls.latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()

    # --- Spin helpers ---------------------------------------------------

    def _spin_until(self, predicate, timeout_s: float) -> bool:
        """Spin the executor until predicate() is true or timeout elapses."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        return predicate()

    # --- Tests ----------------------------------------------------------

    def test_bringup_launches_required_nodes(self) -> None:
        """
        bringup.launch.py brings up the expected node set.

        Polls get_node_names() until every name in EXPECTED_NODE_NAMES
        is present, or NODE_DISCOVERY_TIMEOUT_S elapses. Asserts on the
        missing set so a failure points directly at which node never
        showed up rather than just "not all nodes appeared".
        """
        expected = set(EXPECTED_NODE_NAMES)

        def all_present() -> bool:
            return expected.issubset(set(self.node.get_node_names()))

        self._spin_until(all_present, NODE_DISCOVERY_TIMEOUT_S)

        observed = set(self.node.get_node_names())
        missing = expected - observed
        self.assertFalse(
            missing,
            f"bringup.launch.py did not bring up: {sorted(missing)}; "
            f"observed nodes: {sorted(observed)}"
        )

    def test_bringup_reaches_idle_state(self) -> None:
        """
        /fortis/mission_state latches to IDLE within startup budget.

        mission_state_node publishes its initial state from the
        constructor with TRANSIENT_LOCAL durability, so a subscription
        created after that publish still receives IDLE. If the
        composed system fails to reach a steady state (wrong QoS,
        wrong topic, dead-locked node), no message arrives and the
        assertion below fires with a clear message.
        """
        received: list[String] = []
        self.node.create_subscription(
            String,
            "/fortis/mission_state",
            received.append,
            self.latched_qos,
        )

        self._spin_until(lambda: bool(received), STATE_DELIVERY_TIMEOUT_S)

        self.assertTrue(
            received,
            f"/fortis/mission_state delivered no message within "
            f"{STATE_DELIVERY_TIMEOUT_S}s; mission_state_node is up but "
            "either using a mismatched QoS profile, wrong topic name, "
            "or never reached its initial publish"
        )
        self.assertEqual(
            received[-1].data,
            EXPECTED_STARTUP_STATE,
            f"bringup steady state should be {EXPECTED_STARTUP_STATE}, "
            f"observed {received[-1].data!r}"
        )
