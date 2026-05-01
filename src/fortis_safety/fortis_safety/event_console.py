import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Empty, Bool
from fortis_safety.mission_state_machine import (
    Event,
)
import threading

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

class EventConsole(Node):
    def __init__(self):
        super().__init__("event_console")
        # Latched QoS for state output: late subscribers (e.g. UI started
        # after the node) still get the most recent state on connect.
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.current_state: str = "UNKNOWN"
        self.local_ctx: dict = {}
        
        self.state_sub = self.create_subscription(
            String,
            "/fortis/mission_state",
            self._state_cb,
            latched_qos,
        )
        
        self.event_pubs: dict = {}
        for event in Event:
            topic = f"/fortis/events/{event.name.lower()}"
            self.event_pubs[event.name.lower()] = self.create_publisher(
                Empty, topic, 10
            )
        
        self.ctx_pubs: dict = {}
        for field in CONTEXT_FIELDS:
            topic = f"/fortis/context/{field}"
            self.ctx_pubs[field] = self.create_publisher(Bool, topic, latched_qos)
        
        
    def _state_cb(self, msg: String) -> None:
        self.current_state = msg.data

    def publish_event(self, name: str) -> bool:
        pub = self.event_pubs.get(name)
        if pub is None:
            self.get_logger().error(f"No publisher for event {name}")
            return False
        pub.publish(Empty())
        self.get_logger().info(f"Published event: {name}")
        return True
    
    def publish_context(self, field: str, value: bool) -> bool:
        pub = self.ctx_pubs.get(field)
        if pub is None:
            self.get_logger().error(f"No publisher for context field {field}")
            return False
        msg = Bool()
        msg.data = value
        pub.publish(msg)
        self.get_logger().info(f"Published context: {field} = {value}")
        self.local_ctx[field] = value
        return True
    


def run_cli(node):
    while True:
        try:
            line = input(f"[{node.current_state}] fortis> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()
        args = parts[1:]
        
        if cmd == "state":
            print(node.current_state)
        elif cmd == "event":
            if len(args) != 1:
                print("usage: event <name>")
                continue
            node.publish_event(args[0].lower())
        elif cmd == "set":
            if len(args) != 2:
                print("usage: set <field> <value>")
                continue
            value = parse_bool(args[1])
            if value is None:
                print("value must be a boolean (true/false)")
                continue
            node.publish_context(args[0], value)
        elif cmd == "ctx":
            print(node.local_ctx)
        elif cmd == "events":
            print("Events:", ", ".join(node.event_pubs.keys()))
        elif cmd == "fields":
            print("Context fields:", ", ".join(node.ctx_pubs.keys()))
        elif cmd in ("help", "?"):
            print("Commands:")
            print("  state                 - show current state")
            print("  event <name>         - publish an event")
            print("  set <field> <value> - set a context field (value: true/false)")
            print("  ctx                   - show local context values")
            print("  events                - list available events")
            print("  fields                - list available context fields")
            print("  help                  - show this message")
        elif cmd == "exit":
            break
        else:
            print(f"Unknown command: {cmd}. Type 'help' for a list of commands.")
    
def parse_bool(token: str):
    t = token.lower()
    if t in ("true", "t", "1"):
        return True
    if t in ("false", "f", "0"):
        return False
    return None  # invalid

def main(args=None):
    rclpy.init(args=args)
    node = EventConsole()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()
    try:
        run_cli(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
    
        