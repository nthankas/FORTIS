from motor_base import Motor, MotorStatus
import can
import struct

CMD_HEARTBEAT = 0x01
CMD_SET_INPUT_VELOCITY = 0x0d
CMD_ENCODER_ESTIMATES = 0x09
CMD_SET_AXIS_STATE = 0x07
CMD_E_STOP = 0x02

AXIS_IDLE = 1
AXIS_CLOSED_LOOP_CONTROL = 8

class ODrive_S1(Motor):


    def __init__(self, name: str, can_id: int, max_velocity: float, max_current: float):
        super().__init__(name, can_id, max_velocity, max_current)
        self._bus = can.Bus("can0", interface="virtual")
    
    def _build_can_id(self, cmd_id):
        return (self._can_id << 5) | cmd_id

    def _send_can(self, cmd_id: int, fmt: str, *data):
        can_id = self._build_can_id(cmd_id)
        can_data = struct.pack(fmt, *data)
        self._bus.send(can.Message(arbitration_id=can_id, data=can_data, is_extended_id=False))
    
    def _connect(self):
        self._send_can(CMD_SET_AXIS_STATE, '<I', AXIS_CLOSED_LOOP_CONTROL)
    
    def _disconnect(self):
        self._send_can(CMD_SET_AXIS_STATE, '<I', AXIS_IDLE)

    def _set_velocity(self, velocity):
        self._send_can(CMD_SET_INPUT_VELOCITY, '<ff', velocity, 0.0)

    def _stop_motor(self):
        self._send_can(CMD_SET_INPUT_VELOCITY, '<ff', 0)
        self._send_can(CMD_SET_AXIS_STATE, '<I', AXIS_IDLE)
    
    def _read_position(self):
        while True:
            msg = self._bus.recv(timeout = 1.0)
            if msg is None:
                raise TimeoutError(f"Velocity not read from ODrive CanBus {self._name}, ID: {self._can_id}")
            encoder_id = self._build_can_id(CMD_ENCODER_ESTIMATES)
            if msg.arbitration_id == encoder_id:
                return (struct.unpack('<ff', msg.data))[0]
    
    def _read_velocity(self):
        while True:
            msg = self._bus.recv(timeout = 1.0)
            if msg is None:
                raise TimeoutError(f"Velocity not read from ODrive CanBus {self._name}, ID: {self._can_id}")
            encoder_id = self._build_can_id(CMD_ENCODER_ESTIMATES)
            if msg.arbitration_id == encoder_id:
                return (struct.unpack('<ff', msg.data))[1]

    def _read_current(self):
        return 0.0
