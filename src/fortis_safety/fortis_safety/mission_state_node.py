"""
ROS 2 wrapper node for the FORTIS mission state machine.

Subscribes:
    /fortis/events/<event_name>     std_msgs/Empty   one per Event enum value
    /fortis/context/<field_name>    std_msgs/Bool    one per known ctx key

Publishes:
    /fortis/mission_state           std_msgs/String  current state name,
                                                     latched (transient_local QoS)
                                                     so late subscribers get it

The node is thin by design. All decision logic lives in
MissionStateMachine. This file just wires ROS topics to the machine's
step() method and publishes state changes.

Run with:
    ros2 run fortis_safety mission_state_node

Test from a second terminal:
    ros2 topic echo /fortis/mission_state
    ros2 topic pub --once /fortis/events/start_orbit std_msgs/msg/Empty '{}'
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

from fortis_safety.mission_state_machine import (
    Event,
    MissionStateMachine,
    State,
)


# Context fields the state machine cares about. Each becomes a Bool topic.
# Keep this list in sync with the guard functions in mission_state_machine.py.
CONTEXT_FIELDS = [
    "target_pose_valid",
    "ik_ok",
    "grasp_candidate_ok",
    "gripper_closed",
    "gripper_open",
    "arm_at_home",
    "chassis_at_home",
    "pick_in_contact",
    "operator_ack",
]


class MissionStateNode(Node):
    def __init__(self):
        super().__init__("mission_state_node")

        self.sm = MissionStateMachine()
        self.ctx: dict = {}

        # Latched QoS for state output: late subscribers (e.g. UI started
        # after the node) still get the most recent state on connect.
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.state_pub = self.create_publisher(
            String, "/fortis/mission_state", latched_qos
        )

        # Subscribe to one Empty topic per Event. Topic name = lowercase event.
        # The callback closes over the event so we know which one fired.
        for event in Event:
            topic = f"/fortis/events/{event.name.lower()}"
            self.create_subscription(
                Empty,
                topic,
                self._make_event_callback(event),
                10,
            )

        # Subscribe to one Bool topic per context field.
        for field in CONTEXT_FIELDS:
            topic = f"/fortis/context/{field}"
            self.create_subscription(
                Bool,
                topic,
                self._make_context_callback(field),
                10,
            )

        # Publish initial state so subscribers immediately see IDLE.
        self._publish_state()
        self.get_logger().info(
            f"mission_state_node up. Initial state: {self.sm.current.name}"
        )

    # --- Callback factories -------------------------------------------------
    #
    # Why factories? Each subscription needs its own callback that knows
    # which event/field it represents. A loop variable wouldn't work
    # directly because Python closures capture variables by reference, not
    # value -- by the time the callback fires, the loop variable has moved
    # on. The factory pattern captures the value at function definition.

    def _make_event_callback(self, event: Event):
        def callback(_msg: Empty) -> None:
            before = self.sm.current
            result = self.sm.try_step(event, self.ctx)
            if result.transition_taken:
                self.get_logger().info(
                    f"{before.name} --[{event.name}]--> {result.to_state.name}"
                )
                self._publish_state()
            else:
                self.get_logger().warn(
                    f"Rejected {event.name} in state {before.name} "
                    f"(ctx={self.ctx})"
                )
        return callback

    def _make_context_callback(self, field: str):
        def callback(msg: Bool) -> None:
            self.ctx[field] = msg.data
        return callback

    # --- Helpers ------------------------------------------------------------

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self.sm.current.name
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()