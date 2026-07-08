"""
Serial bridge between the FORTIS arm Teensy 4.1 and the ROS 2 graph.

Owns the USB-CDC link to the firmware described in
``firmware/teensy/PROTOCOL.md`` (and mocked by ``tools/mock_teensy.py``):

* a reconnect-forever background reader thread parses inbound frames;
* rclpy timers send CMD_HEARTBEAT (<=100 ms) and poll CMD_GET_STATUS;
* RSP_STATUS is republished as fortis_msgs/ArmStatus (latched),
  sensor_msgs/JointState (calibrated to rad / metres) and a
  diagnostic_msgs/DiagnosticArray with decoded fault names;
* EVT_FAULT is forwarded as std_msgs/Empty on /fortis/events/fault;
* /fortis/arm/command (sensor_msgs/JointState, partial names allowed) is
  translated to CMD_SET_JOINT_TARGETS;
* enable / disable / home / clear_faults are std_srvs/Trigger services.

The node never raises on a missing serial port: it publishes
ArmStatus.connected=false and keeps retrying to open the port.
"""

from __future__ import annotations

import dataclasses
import struct
import threading

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from fortis_comms.qos_profiles import latched_qos_profile
from fortis_msgs.msg import ArmStatus

from fortis_arm.teensy_protocol import (
    HARD_FAULTS,
    PROTO_MAJOR,
    Ack,
    BootEvent,
    ErrCode,
    FaultEvent,
    FrameParser,
    JointTargets,
    MsgType,
    Nak,
    StateBit,
    Status,
    encode,
    fault_names,
)

ARM_STATUS_TOPIC = "/fortis/arm/status"
JOINT_STATES_TOPIC = "/joint_states"
DIAGNOSTICS_TOPIC = "/diagnostics"
FAULT_EVENT_TOPIC = "/fortis/events/fault"
ARM_COMMAND_TOPIC = "/fortis/arm/command"

#: JointState names published on /joint_states, in publish order.
ARM_JOINT_NAMES = (
    "joint_j1",
    "joint_j2",
    "joint_j3",
    "joint_j4",
    "gripper_left_joint",
    "gripper_right_joint",
)

DEFAULT_STEPS_PER_RAD_J1 = 12732.4
DEFAULT_STEPS_PER_RAD_J2 = 12732.4
DEFAULT_STEPS_PER_RAD_J3 = 6875.5

#: CMD_SET_JOINT_TARGETS flag bit0 = clamp_to_limits (PROTOCOL.md 3.4).
_FLAG_CLAMP_TO_LIMITS = 0x01
#: CMD_HOME_REQUEST mask 0x08 = J4 only, the sole bit real firmware ACKs
#: (steppers/gripper NAK ERR_NOT_IMPLEMENTED). PROTOCOL.md 3.6.
_HOME_MASK_J4 = 0x08
#: CMD_CLEAR_FAULTS mask: clear every latched bit.
_CLEAR_ALL_MASK = 0xFFFF
#: Gripper servo pulse hard limits from the protocol (PROTOCOL.md 3.4).
_GRIPPER_US_MIN = 1000.0
_GRIPPER_US_MAX = 2000.0
#: Timeout for a service/command to hear back an ACK/NAK.
_ACK_TIMEOUT_S = 1.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclasses.dataclass
class _Pending:
    event: threading.Event
    ack: bool = False
    err: "int | None" = None


class TeensyBridgeNode(Node):
    """ROS 2 bridge owning the arm Teensy serial link.

    Runs a reconnect-forever reader thread plus heartbeat and status
    timers, republishes decoded status, relays /fortis/arm/command to
    CMD_SET_JOINT_TARGETS, and exposes enable / disable / home /
    clear_faults as std_srvs/Trigger services.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__("teensy_bridge", **kwargs)

        self._serial_port = self._decls("serial_port", "/dev/ttyACM0")
        self._baud = self._decli("baud", 1000000)
        status_rate = self._declf("status_rate_hz", 20.0)
        hb_rate = self._declf("heartbeat_rate_hz", 20.0)

        self._steps_per_rad_j1 = self._declf(
            "steps_per_rad_j1", DEFAULT_STEPS_PER_RAD_J1)
        self._steps_per_rad_j2 = self._declf(
            "steps_per_rad_j2", DEFAULT_STEPS_PER_RAD_J2)
        self._steps_per_rad_j3 = self._declf(
            "steps_per_rad_j3", DEFAULT_STEPS_PER_RAD_J3)
        self._j4_us_center = self._declf("j4_us_center", 1400.0)
        self._j4_us_per_rad = self._declf("j4_us_per_rad", 382.0)
        self._j4_us_min = self._decli("j4_us_min", 800)
        self._j4_us_max = self._decli("j4_us_max", 2000)
        self._gripper_open_us = self._decli("gripper_open_us", 1900)
        self._gripper_closed_us = self._decli("gripper_closed_us", 1100)
        self._gripper_stroke_m = self._declf("gripper_stroke_m", 0.027)
        self._max_sps = [
            self._declf("max_sps_j1", 8000.0),
            self._declf("max_sps_j2", 8000.0),
            self._declf("max_sps_j3", 8000.0),
        ]

        # Publishers.
        self._status_pub = self.create_publisher(
            ArmStatus, ARM_STATUS_TOPIC, latched_qos_profile())
        self._joint_pub = self.create_publisher(
            JointState, JOINT_STATES_TOPIC, 10)
        self._diag_pub = self.create_publisher(
            DiagnosticArray, DIAGNOSTICS_TOPIC, 10)
        self._fault_pub = self.create_publisher(Empty, FAULT_EVENT_TOPIC, 10)

        # Serial / protocol state.
        self._serial = None
        self._connected = False
        self._parser = FrameParser()
        self._seq = 0
        self._seq_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._pending: "dict[int, _Pending]" = {}
        self._pending_lock = threading.Lock()
        self._last_status: "Status | None" = None
        self._last_targets = JointTargets()
        self._have_targets = False
        self._stop = threading.Event()

        # Callback groups: keep timers off the service/subscription group so
        # a blocked ACK-wait cannot starve the heartbeat under a
        # MultiThreadedExecutor.
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._io_group = ReentrantCallbackGroup()

        self.create_subscription(
            JointState, ARM_COMMAND_TOPIC, self._on_command, 10,
            callback_group=self._io_group)

        self.create_service(
            Trigger, "~/enable", self._srv_enable,
            callback_group=self._io_group)
        self.create_service(
            Trigger, "~/disable", self._srv_disable,
            callback_group=self._io_group)
        self.create_service(
            Trigger, "~/home", self._srv_home,
            callback_group=self._io_group)
        self.create_service(
            Trigger, "~/clear_faults", self._srv_clear_faults,
            callback_group=self._io_group)

        self.create_timer(
            1.0 / max(hb_rate, 1.0), self._on_heartbeat,
            callback_group=self._timer_group)
        self.create_timer(
            1.0 / max(status_rate, 1.0), self._on_status_timer,
            callback_group=self._timer_group)

        self._reader = threading.Thread(
            target=self._serial_loop, name="teensy_reader", daemon=True)
        self._reader.start()

        self.get_logger().info(
            f"teensy_bridge up; opening {self._serial_port} @ {self._baud}")

    # --- parameter helpers ------------------------------------------------

    def _decls(self, name: str, default: str) -> str:
        return str(self.declare_parameter(name, default).value)

    def _decli(self, name: str, default: int) -> int:
        return int(self.declare_parameter(name, default).value)

    def _declf(self, name: str, default: float) -> float:
        return float(self.declare_parameter(name, default).value)

    # --- serial I/O -------------------------------------------------------

    def _serial_loop(self) -> None:
        while not self._stop.is_set():
            try:
                port = serial.Serial(self._serial_port, self._baud, timeout=0.05)
            except (OSError, serial.SerialException) as exc:
                self._connected = False
                self.get_logger().warn(
                    f"cannot open {self._serial_port}: {exc}; retrying",
                    throttle_duration_sec=5.0)
                self._stop.wait(1.0)
                continue
            self._serial = port
            self._parser = FrameParser()
            self._connected = True
            self.get_logger().info(f"serial open on {self._serial_port}")
            try:
                while not self._stop.is_set():
                    data = port.read(256)
                    if data:
                        for frame in self._parser.feed(data):
                            self._dispatch(frame)
            except (OSError, serial.SerialException) as exc:
                self.get_logger().warn(f"serial link error: {exc}")
            finally:
                self._connected = False
                self._serial = None
                try:
                    port.close()
                except (OSError, serial.SerialException):
                    pass
                self._publish_status(self._last_status or Status(), False)

    def _next_seq(self) -> int:
        with self._seq_lock:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFF
            return seq

    def _write_frame(self, seq: int, msg_type: int, payload: bytes) -> bool:
        port = self._serial
        if port is None:
            return False
        frame = encode(seq, msg_type, payload)
        try:
            with self._write_lock:
                port.write(frame)
            return True
        except (OSError, serial.SerialException):
            return False

    def _request(self, msg_type: int, payload: bytes = b"") -> "tuple[bool, int | None]":
        """Send a command and wait for its ACK/NAK; return (ok, err)."""
        if not self._connected:
            return (False, None)
        seq = self._next_seq()
        pending = _Pending(event=threading.Event())
        with self._pending_lock:
            self._pending[seq] = pending
        if not self._write_frame(seq, msg_type, payload):
            with self._pending_lock:
                self._pending.pop(seq, None)
            return (False, None)
        got = pending.event.wait(_ACK_TIMEOUT_S)
        with self._pending_lock:
            self._pending.pop(seq, None)
        if not got:
            return (False, None)
        return (pending.ack, pending.err)

    def _resolve(self, seq: int, ack: bool, err: "int | None") -> None:
        with self._pending_lock:
            pending = self._pending.get(seq)
        if pending is not None:
            pending.ack = ack
            pending.err = err
            pending.event.set()

    # --- frame dispatch ---------------------------------------------------

    def _dispatch(self, frame) -> None:
        try:
            self._dispatch_inner(frame)
        except (struct.error, IndexError) as exc:
            self.get_logger().warn(
                f"malformed frame type=0x{frame.msg_type:02X}: {exc}")

    def _dispatch_inner(self, frame) -> None:
        t = frame.msg_type
        if t == int(MsgType.RSP_STATUS):
            self._on_status_frame(Status.unpack(frame.payload))
        elif t == int(MsgType.RSP_ACK):
            ack = Ack.unpack(frame.payload)
            self._resolve(ack.acked_seq, True, None)
        elif t == int(MsgType.RSP_NAK):
            nak = Nak.unpack(frame.payload)
            self._resolve(nak.nak_seq, False, nak.err_code)
        elif t == int(MsgType.EVT_FAULT):
            evt = FaultEvent.unpack(frame.payload)
            self.get_logger().warn(
                f"EVT_FAULT rising={fault_names(evt.new_bits)} "
                f"flags={fault_names(evt.current_flags)}")
            self._fault_pub.publish(Empty())
        elif t == int(MsgType.EVT_BOOT):
            boot = BootEvent.unpack(frame.payload)
            if boot.proto_major != PROTO_MAJOR:
                self.get_logger().warn(
                    f"teensy proto major {boot.proto_major} != {PROTO_MAJOR}")
            else:
                self.get_logger().info(
                    f"teensy boot proto {boot.proto_major}.{boot.proto_minor} "
                    f"slot={boot.eeprom_slot} recovered={boot.recovered}")

    def _on_status_frame(self, st: Status) -> None:
        self._last_status = st
        if not self._have_targets:
            self._last_targets = JointTargets(
                j1_steps=st.j1_steps, j2_steps=st.j2_steps,
                j3_steps=st.j3_steps, j4_us=st.j4_us,
                gripper_us=st.gripper_us)
            self._have_targets = True
        self._publish_status(st, True)
        self._publish_joint_states(st)
        self._publish_diagnostics(st, True)

    # --- publishers -------------------------------------------------------

    def _publish_status(self, st: Status, connected: bool) -> None:
        msg = ArmStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.connected = connected
        msg.joint_steps = [
            int(st.j1_steps), int(st.j2_steps), int(st.j3_steps)]
        msg.j4_us = int(st.j4_us)
        msg.gripper_us = int(st.gripper_us)
        msg.fault_flags = int(st.fault_flags)
        msg.state_bits = int(st.state_bits)
        msg.uptime_ms = int(st.uptime_ms)
        msg.mcu_temp_c = st.mcu_temp_c10 / 10.0
        self._status_pub.publish(msg)

    def _publish_joint_states(self, st: Status) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(ARM_JOINT_NAMES)
        grip = self._gripper_us_to_m(st.gripper_us)
        js.position = [
            st.j1_steps / self._steps_per_rad_j1,
            st.j2_steps / self._steps_per_rad_j2,
            st.j3_steps / self._steps_per_rad_j3,
            (st.j4_us - self._j4_us_center) / self._j4_us_per_rad,
            grip,
            -grip,
        ]
        self._joint_pub.publish(js)

    def _publish_diagnostics(self, st: Status, connected: bool) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "teensy_bridge: arm link"
        status.hardware_id = self._serial_port
        flags = st.fault_flags
        if not connected:
            status.level = DiagnosticStatus.WARN
            status.message = "serial link down"
        elif flags & int(HARD_FAULTS):
            status.level = DiagnosticStatus.ERROR
            status.message = "fault: " + ", ".join(fault_names(flags))
        elif flags:
            status.level = DiagnosticStatus.WARN
            status.message = "fault: " + ", ".join(fault_names(flags))
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ok"
        enabled = bool(st.state_bits & int(StateBit.ENABLED))
        status.values = [
            KeyValue(key="connected", value=str(connected)),
            KeyValue(key="faults", value=",".join(fault_names(flags)) or "none"),
            KeyValue(key="state_bits", value=hex(st.state_bits)),
            KeyValue(key="enabled", value=str(enabled)),
            KeyValue(key="uptime_ms", value=str(st.uptime_ms)),
            KeyValue(key="mcu_temp_c", value=f"{st.mcu_temp_c10 / 10.0:.1f}"),
            KeyValue(key="crc_errors", value=str(self._parser.crc_errors)),
        ]
        arr.status = [status]
        self._diag_pub.publish(arr)

    def _gripper_us_to_m(self, us: float) -> float:
        span = self._gripper_open_us - self._gripper_closed_us
        if span == 0:
            return 0.0
        frac = (us - self._gripper_closed_us) / span
        return frac * self._gripper_stroke_m

    def _m_to_gripper_us(self, meters: float) -> int:
        span = self._gripper_open_us - self._gripper_closed_us
        if self._gripper_stroke_m == 0.0 or span == 0:
            return int(self._gripper_closed_us)
        frac = _clamp(meters / self._gripper_stroke_m, 0.0, 1.0)
        us = self._gripper_closed_us + frac * span
        return int(round(_clamp(us, _GRIPPER_US_MIN, _GRIPPER_US_MAX)))

    # --- timers -----------------------------------------------------------

    def _on_heartbeat(self) -> None:
        if self._connected:
            self._write_frame(
                self._next_seq(), int(MsgType.CMD_HEARTBEAT), b"")

    def _on_status_timer(self) -> None:
        if self._connected:
            self._write_frame(
                self._next_seq(), int(MsgType.CMD_GET_STATUS), b"")
        else:
            st = self._last_status or Status()
            self._publish_status(st, False)
            self._publish_diagnostics(st, False)

    # --- command subscription ---------------------------------------------

    def _on_command(self, msg: JointState) -> None:
        base = self._last_targets
        updates: "dict[str, int]" = {}
        for name, pos in zip(list(msg.name), list(msg.position)):
            pos = float(pos)
            if name == "joint_j1":
                updates["j1_steps"] = int(round(pos * self._steps_per_rad_j1))
            elif name == "joint_j2":
                updates["j2_steps"] = int(round(pos * self._steps_per_rad_j2))
            elif name == "joint_j3":
                updates["j3_steps"] = int(round(pos * self._steps_per_rad_j3))
            elif name == "joint_j4":
                us = self._j4_us_center + pos * self._j4_us_per_rad
                us = _clamp(us, float(self._j4_us_min), float(self._j4_us_max))
                updates["j4_us"] = int(round(us))
            elif name == "gripper_left_joint":
                updates["gripper_us"] = self._m_to_gripper_us(pos)
            elif name == "gripper_right_joint":
                updates["gripper_us"] = self._m_to_gripper_us(-pos)
        if not updates:
            return
        tgt = dataclasses.replace(
            base, flags=_FLAG_CLAMP_TO_LIMITS, **updates)
        self._last_targets = tgt
        self._request(int(MsgType.CMD_SET_JOINT_TARGETS), tgt.pack())

    # --- services ---------------------------------------------------------

    def _trigger(self, msg_type: int, payload: bytes, response):
        if not self._connected:
            response.success = False
            response.message = "teensy serial link down"
            return response
        ok, err = self._request(msg_type, payload)
        if ok:
            response.success = True
            response.message = "ACK"
        elif err is not None:
            response.success = False
            response.message = f"NAK: {self._err_name(err)}"
        else:
            response.success = False
            response.message = "no response from teensy (timeout)"
        return response

    @staticmethod
    def _err_name(err: int) -> str:
        try:
            return ErrCode(err).name
        except ValueError:
            return f"0x{err:02X}"

    def _srv_enable(self, request, response):
        return self._trigger(int(MsgType.CMD_ENABLE), b"", response)

    def _srv_disable(self, request, response):
        return self._trigger(int(MsgType.CMD_DISABLE), b"", response)

    def _srv_home(self, request, response):
        return self._trigger(
            int(MsgType.CMD_HOME_REQUEST), bytes([_HOME_MASK_J4]), response)

    def _srv_clear_faults(self, request, response):
        return self._trigger(
            int(MsgType.CMD_CLEAR_FAULTS),
            struct.pack("<H", _CLEAR_ALL_MASK), response)

    # --- lifecycle --------------------------------------------------------

    def destroy_node(self):
        self._stop.set()
        port = self._serial
        if port is not None:
            try:
                port.close()
            except (OSError, serial.SerialException):
                pass
        if self._reader.is_alive():
            self._reader.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeensyBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
