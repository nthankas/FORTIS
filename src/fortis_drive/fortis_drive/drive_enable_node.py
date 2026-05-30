"""
drive_enable_node -- UI-facing arm/disarm for the X-drive.

Lets the operator arm or disarm the wheel drive from the UI with a single
std_msgs/Bool on /fortis/commands/drive_enable, without ever touching
ros2_control internals. On enable it activates the
wheel_velocity_controller (which puts the four ODrive S1s into
CLOSED_LOOP_CONTROL); on disable it deactivates it (idles the motors).

Why this exists
---------------
Arming is a controller_manager *service* (switch_controller), not a
topic, so it cannot be driven directly from a telemetry UI like Foxglove.
This node wraps that service behind a plain Bool topic so the UI can arm
with a button *publish* -- the most reliable UI primitive -- and the
operator never sees a controller name. It keeps FORTIS's two-lock safety:
arming (this node) and the mission gate (drive_node + the FSM) remain
separate, deliberate operator actions.

Subscribes
----------
    /fortis/commands/drive_enable   std_msgs/Bool
        True  -> activate wheel_velocity_controller (arm)
        False -> deactivate wheel_velocity_controller (disarm)

Publishes
---------
    /fortis/drive/armed   std_msgs/Bool   (latched, TRANSIENT_LOCAL)
        Current armed state, updated from the switch_controller *result*
        so the UI reflects what actually happened, not just the request.

Calls
-----
    /controller_manager/switch_controller
        controller_manager_msgs/srv/SwitchController
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

from controller_manager_msgs.srv import SwitchController

WHEEL_CONTROLLER = "wheel_velocity_controller"
SWITCH_SERVICE = "/controller_manager/switch_controller"
ENABLE_TOPIC = "/fortis/commands/drive_enable"
ARMED_TOPIC = "/fortis/drive/armed"
SERVICE_WAIT_S = 2.0


class DriveEnableNode(Node):
    """Translate a Bool arm/disarm command into a controller switch."""

    def __init__(self) -> None:
        super().__init__("drive_enable_node")

        # Latched so a UI connecting later immediately learns the armed state.
        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        self._armed_pub = self.create_publisher(Bool, ARMED_TOPIC, latched)
        self._publish_armed(False)

        self._switch_cli = self.create_client(SwitchController, SWITCH_SERVICE)
        self.create_subscription(Bool, ENABLE_TOPIC, self._on_enable, 10)

        self.get_logger().info(
            f"drive_enable_node ready; publish std_msgs/Bool on {ENABLE_TOPIC} to arm/disarm."
        )

    def _publish_armed(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self._armed_pub.publish(msg)

    def _on_enable(self, msg: Bool) -> None:
        arm = bool(msg.data)

        if not self._switch_cli.service_is_ready():
            if not self._switch_cli.wait_for_service(timeout_sec=SERVICE_WAIT_S):
                self.get_logger().error(
                    f"{SWITCH_SERVICE} unavailable -- is controller_manager running?"
                )
                return

        req = SwitchController.Request()
        if arm:
            req.activate_controllers = [WHEEL_CONTROLLER]
            req.strictness = SwitchController.Request.STRICT
        else:
            req.deactivate_controllers = [WHEEL_CONTROLLER]
            # Best-effort on disarm: idling an already-inactive controller
            # should never error out the operator's "stop" action.
            req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True

        future = self._switch_cli.call_async(req)
        future.add_done_callback(lambda f: self._on_switch_result(f, arm))

    def _on_switch_result(self, future, arm: bool) -> None:
        try:
            ok = future.result().ok
        except Exception as exc:  # noqa: BLE001 - surface any service failure
            self.get_logger().error(f"switch_controller call failed: {exc}")
            return

        if ok:
            self.get_logger().info(f"drive {'ARMED' if arm else 'DISARMED'}.")
            self._publish_armed(arm)
        else:
            action = "activate" if arm else "deactivate"
            self.get_logger().warning(
                f"switch_controller rejected {action} of {WHEEL_CONTROLLER}."
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveEnableNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
