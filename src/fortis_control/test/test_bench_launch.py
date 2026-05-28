"""
launch_testing integration test for bench_one_motor.launch.py.

What this verifies end-to-end, with mock hardware:

  1. controller_manager loads against the rendered URDF (xacro + YAML +
     plugin path all agree).
  2. joint_state_broadcaster activates and publishes /joint_states with
     fl_wheel_joint present.
  3. wheel_velocity_controller can be activated against the joint.
  4. A Float64MultiArray [v] on /wheel_velocity_controller/commands is
     applied to the mocked joint and feeds back through /joint_states.

This is the closest thing to a hardware bench test we can run without a
real ODrive + CAN bus. It is what tells us "the wiring is right" before
any motor is plugged in.

Why use_mock_hardware:=true
---------------------------
mock_components/GenericSystem with calculate_dynamics:=true mirrors the
commanded velocity back into the velocity state interface (integrating
position from it). That gives us a deterministic, hardware-free target
to assert against. The real ODrive plugin would refuse to load without
a calibrated motor on the bus.

Run inside the dev container:
    cd /workspace
    colcon test --packages-select fortis_control \
        --event-handlers console_direct+
"""

from __future__ import annotations

import time
import unittest

import launch_testing.actions
import pytest
import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# Hard ceilings. Generous because controller_manager startup is heavier
# than rclpy-only round trips and CI boxes are slow.
CONTROLLER_LIST_TIMEOUT_S: float = 30.0
ACTIVATION_TIMEOUT_S: float = 15.0
JOINT_STATE_TIMEOUT_S: float = 15.0


@pytest.mark.launch_test
def generate_test_description():
    """
    Bring up bench_one_motor.launch.py under mock hardware.

    The included launch resolves to ros2_control_node +
    robot_state_publisher + the two spawners exactly the way the bench
    operator would invoke it; we override use_mock_hardware so no CAN
    bus is required.
    """
    bench_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("fortis_control"),
                "launch",
                "bench_one_motor.launch.py",
            ])
        ),
        launch_arguments={
            "use_mock_hardware": "true",
        }.items(),
    )

    return LaunchDescription([
        bench_launch,
        # ReadyToTest must be the last action so launch_testing knows the
        # included stack is up before tests start running. There is no
        # other handshake -- we rely on the polling in the tests below.
        launch_testing.actions.ReadyToTest(),
    ])


# --- Helpers ----------------------------------------------------------------


class _Probe:
    """Single rclpy node that subscribes to /joint_states and calls services."""

    def __init__(self) -> None:
        self.node = rclpy.create_node("fortis_control_bench_probe")
        self.joint_states: list[JointState] = []
        self.node.create_subscription(
            JointState, "/joint_states", self.joint_states.append, 10
        )
        self.cmd_pub = self.node.create_publisher(
            Float64MultiArray,
            "/wheel_velocity_controller/commands",
            10,
        )
        self.list_cli = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switch_cli = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

    def spin(self, seconds: float = 0.1) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_for_controller(self, name: str, timeout_s: float) -> dict:
        """Poll list_controllers until `name` is present; return its entry."""
        end = time.monotonic() + timeout_s
        assert self.list_cli.wait_for_service(timeout_sec=timeout_s), (
            "/controller_manager/list_controllers never came up"
        )
        while time.monotonic() < end:
            fut = self.list_cli.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=2.0)
            if fut.result() is not None:
                for c in fut.result().controller:
                    if c.name == name:
                        return {"name": c.name, "state": c.state,
                                "type": c.type}
            self.spin(0.5)
        raise AssertionError(
            f"controller {name!r} never appeared in list_controllers within "
            f"{timeout_s:.1f}s"
        )

    def activate_controller(self, name: str, timeout_s: float) -> None:
        """Send a switch_controller request to activate `name`; wait for response."""
        self._switch(activate=[name], deactivate=[], timeout_s=timeout_s)

    def deactivate_controller(self, name: str, timeout_s: float) -> None:
        """Send a switch_controller request to deactivate `name`."""
        self._switch(activate=[], deactivate=[name], timeout_s=timeout_s)

    def _switch(self, activate: list[str], deactivate: list[str],
                timeout_s: float) -> None:
        assert self.switch_cli.wait_for_service(timeout_sec=timeout_s)
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.STRICT
        fut = self.switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout_s)
        assert fut.result() is not None, (
            f"switch_controller(activate={activate}, "
            f"deactivate={deactivate}) call timed out"
        )
        assert fut.result().ok, (
            f"switch_controller(activate={activate}, "
            f"deactivate={deactivate}) returned ok=False"
        )

    def wait_for_joint_state_with(self, joint_name: str,
                                  timeout_s: float) -> JointState:
        """Drain /joint_states until one carrying `joint_name` arrives."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self.spin(0.2)
            for js in reversed(self.joint_states):
                if joint_name in js.name:
                    return js
        raise AssertionError(
            f"no /joint_states with {joint_name!r} arrived within "
            f"{timeout_s:.1f}s "
            f"(got {len(self.joint_states)} messages total)"
        )

    def cleanup(self) -> None:
        self.node.destroy_node()


# --- Tests ------------------------------------------------------------------


class TestBenchLaunch(unittest.TestCase):
    """
    End-to-end test against bench_one_motor.launch.py with mock hardware.

    Each test method holds its own rclpy session because launch_testing
    runs them in the same process and a leaked TRANSIENT_LOCAL writer
    can cross test boundaries otherwise (the same fix
    test_drive_node.py applied at unit level).
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.probe = _Probe()
        # Wait for controller_manager + the spawners to settle. The
        # joint_state_broadcaster spawn is in the launch chain; we just
        # poll for the controller list to include it.
        self.probe.spin(2.0)

    def tearDown(self):
        self.probe.cleanup()

    def test_joint_state_broadcaster_is_active(self):
        """joint_state_broadcaster must reach 'active' on its own (no --inactive)."""
        info = self.probe.wait_for_controller(
            "joint_state_broadcaster",
            timeout_s=CONTROLLER_LIST_TIMEOUT_S,
        )
        # joint_state_broadcaster spawns active by default in the launch
        # file. If this regresses to 'inactive', /joint_states is silent
        # and downstream nodes (robot_localization, the bench operator's
        # `ros2 topic echo`) see nothing.
        self.assertEqual(info["state"], "active",
                         f"joint_state_broadcaster state: {info['state']}")

    def test_wheel_velocity_controller_loads_inactive(self):
        """
        wheel_velocity_controller must be loaded but inactive at launch.

        Bench safety contract: the controller does not claim the
        velocity interface until the operator explicitly activates it.
        Auto-activation would mean a launch could spin a motor before
        the operator is ready.
        """
        info = self.probe.wait_for_controller(
            "wheel_velocity_controller",
            timeout_s=CONTROLLER_LIST_TIMEOUT_S,
        )
        self.assertEqual(
            info["state"], "inactive",
            f"wheel_velocity_controller must start inactive; got {info['state']}"
        )
        self.assertEqual(
            info["type"],
            "velocity_controllers/JointGroupVelocityController",
        )

    def test_joint_states_contain_fl_wheel_joint(self):
        """The mocked single-wheel hardware must expose fl_wheel_joint state."""
        js = self.probe.wait_for_joint_state_with(
            "fl_wheel_joint", timeout_s=JOINT_STATE_TIMEOUT_S,
        )
        idx = js.name.index("fl_wheel_joint")
        # Position field present (mock starts at 0; calculate_dynamics
        # integrates from commanded velocity).
        self.assertTrue(len(js.position) > idx)
        self.assertTrue(len(js.velocity) > idx)

    def test_commanded_velocity_reaches_joint_state_under_mock(self):
        """
        Send Float64MultiArray, observe the mock's velocity state.

        mock_components/GenericSystem with calculate_dynamics:=true is
        deterministic: a commanded velocity is mirrored back through
        the velocity state interface. This test verifies the entire
        loop -- controller_manager.update -> hardware.write ->
        hardware.read -> joint_state_broadcaster -- without any real
        hardware in the path.

        If this test passes, then on the bench, the only failure modes
        remaining are: (a) CAN bus / cabling, (b) calibration not done,
        (c) odrive_ros2_control plugin behaviour vs mock. Mock behaviour
        is, by definition, the spec the real plugin is replacing.

        Cleanup: deactivates the controller before returning so the
        loads_inactive test (which checks startup state) is not
        polluted by leftover activation. unittest runs tests
        alphabetically and there is only one launch, so without this
        explicit teardown the controller stays active across tests.
        """
        # Make sure controller is loaded before activating.
        self.probe.wait_for_controller(
            "wheel_velocity_controller",
            timeout_s=CONTROLLER_LIST_TIMEOUT_S,
        )
        self.probe.activate_controller(
            "wheel_velocity_controller",
            timeout_s=ACTIVATION_TIMEOUT_S,
        )
        try:
            # Drain pre-command joint states so we can observe the
            # transition cleanly.
            self.probe.joint_states.clear()

            # Push a steady velocity command for several controller cycles.
            cmd = Float64MultiArray()
            cmd.data = [1.5]  # rad/s, single-wheel bench
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                self.probe.cmd_pub.publish(cmd)
                self.probe.spin(0.05)

            # Find the most recent joint state with fl_wheel_joint and
            # verify its velocity tracks the command.
            latest = None
            for js in reversed(self.probe.joint_states):
                if "fl_wheel_joint" in js.name:
                    latest = js
                    break
            self.assertIsNotNone(
                latest,
                "no /joint_states with fl_wheel_joint received after activation",
            )
            idx = latest.name.index("fl_wheel_joint")
            observed_vel = latest.velocity[idx]
            # Tolerance is generous: mock dynamics adds no noise but
            # the measurement is timing-sensitive (last published state
            # before spin ends). 0.2 rad/s window is well inside the
            # gap between 0 (not commanded) and 1.5 (commanded) so a
            # false pass under noise is implausible.
            self.assertAlmostEqual(
                observed_vel, 1.5, delta=0.2,
                msg=(f"commanded 1.5 rad/s, observed {observed_vel} rad/s "
                     f"in fl_wheel_joint velocity")
            )
        finally:
            # Restore startup state so the next test sees the launch
            # as launched.
            self.probe.deactivate_controller(
                "wheel_velocity_controller",
                timeout_s=ACTIVATION_TIMEOUT_S,
            )


# --- Post-shutdown test (runs after the launch is torn down) ------------------


@launch_testing.post_shutdown_test()
class TestBenchLaunchShutdown(unittest.TestCase):
    """controller_manager must exit cleanly when the launch tears down."""

    def test_controller_manager_exits_cleanly(self, proc_info):
        # If controller_manager crashed mid-test, this surfaces it as a
        # post-shutdown failure rather than a confusing mid-test assert.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
