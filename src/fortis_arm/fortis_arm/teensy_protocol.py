"""
Pure-stdlib codec for the FORTIS Teensy 4.1 binary serial protocol.

Mirrors ``firmware/teensy/PROTOCOL.md`` and ``tools/mock_teensy.py`` exactly
so the teensy_bridge node and its tests share one framing implementation.
No rclpy, no pyserial -- only struct / enum / dataclasses.

Frame layout, little-endian throughout::

    0xA5 | LEN | SEQ | TYPE | PAYLOAD[LEN] | CRC_LO | CRC_HI | 0x5A

CRC-16-CCITT (poly 0x1021, init 0xFFFF, no reflection, no final XOR) is
computed over ``[SEQ, TYPE, PAYLOAD]`` -- NOT over LEN. This matches the mock
firmware, which is the authority the tests run against.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag

FRAME_START = 0xA5
FRAME_END = 0x5A
MAX_PAYLOAD = 240
PROTO_MAJOR = 1


class MsgType(IntEnum):
    """Message type IDs (PROTOCOL.md section 3)."""

    CMD_HEARTBEAT = 0x01
    CMD_GET_STATUS = 0x02
    CMD_ENABLE = 0x03
    CMD_DISABLE = 0x04
    CMD_SET_JOINT_TARGETS = 0x10
    CMD_SET_JOINT_VELOCITIES = 0x11
    CMD_HOME_REQUEST = 0x12
    CMD_CLEAR_FAULTS = 0x13
    RSP_ACK = 0x80
    RSP_NAK = 0x81
    RSP_STATUS = 0x82
    EVT_FAULT = 0xC0
    EVT_BOOT = 0xC1


class ErrCode(IntEnum):
    """NAK error codes (PROTOCOL.md section 5)."""

    NONE = 0x00
    CRC = 0x01
    LENGTH = 0x02
    UNKNOWN_TYPE = 0x03
    BAD_PARAMETER = 0x04
    DISABLED = 0x05
    FAULT_ACTIVE = 0x06
    NOT_HOMED = 0x07
    NOT_IMPLEMENTED = 0x08
    BUSY = 0x09


class Fault(IntFlag):
    """Latched fault bits (PROTOCOL.md section 4)."""

    HEARTBEAT_TIMEOUT = 1 << 0
    DRIVER_ALARM_J1 = 1 << 1
    DRIVER_ALARM_J2 = 1 << 2
    DRIVER_ALARM_J3 = 1 << 3
    EEPROM_CRC = 1 << 4
    OVER_TEMP = 1 << 5
    UNCALIBRATED = 1 << 6
    E_STOP = 1 << 7
    FRAME_OVERFLOW = 1 << 8
    BAD_PARAMETER = 1 << 9


class StateBit(IntFlag):
    """State-word bits (PROTOCOL.md section 3.10)."""

    ENABLED = 1 << 0
    MOVING = 1 << 1
    HOMED_J1 = 1 << 2
    HOMED_J2 = 1 << 3
    HOMED_J3 = 1 << 4


#: Faults that inhibit motion and warrant a DiagnosticStatus.ERROR. The
#: remainder (uncalibrated, frame overflow, bad parameter) are WARN-level.
HARD_FAULTS = (
    Fault.HEARTBEAT_TIMEOUT
    | Fault.DRIVER_ALARM_J1
    | Fault.DRIVER_ALARM_J2
    | Fault.DRIVER_ALARM_J3
    | Fault.EEPROM_CRC
    | Fault.OVER_TEMP
    | Fault.E_STOP
)


def crc16_ccitt(data: bytes) -> int:
    """Return CRC-16-CCITT (poly 0x1021, init 0xFFFF) over ``data``."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode(seq: int, msg_type: int, payload: bytes = b"") -> bytes:
    """Encode one framed message; SEQ is masked to 8 bits."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD}")
    inner = bytes([seq & 0xFF, msg_type & 0xFF]) + payload
    crc = crc16_ccitt(inner)
    return (
        bytes([FRAME_START, len(payload), seq & 0xFF, msg_type & 0xFF])
        + payload
        + bytes([crc & 0xFF, (crc >> 8) & 0xFF, FRAME_END])
    )


def fault_names(flags: int) -> list[str]:
    """Return the names of the set fault bits, low bit first."""
    return [f.name for f in Fault if flags & int(f)]


@dataclass(frozen=True)
class Frame:
    """A decoded, CRC-validated frame."""

    seq: int
    msg_type: int
    payload: bytes


class FrameParser:
    """Incremental byte-stream frame decoder mirroring the firmware.

    ``feed`` returns the frames completed by the chunk. Bad-CRC and
    bad-END frames are dropped and counted in ``crc_errors``; the parser
    resyncs by discarding bytes until the next ``0xA5`` start byte.
    """

    def __init__(self) -> None:
        self.crc_errors = 0
        self._reset()

    def _reset(self) -> None:
        self._phase = "WAIT_START"
        self._len = 0
        self._seq = 0
        self._type = 0
        self._payload = bytearray()
        self._crc_lo = 0
        self._crc = 0

    def feed(self, data: bytes) -> list[Frame]:
        """Consume ``data`` and return any complete valid frames."""
        frames: list[Frame] = []
        for byte in data:
            phase = self._phase
            if phase == "WAIT_START":
                if byte == FRAME_START:
                    self._phase = "READ_LEN"
            elif phase == "READ_LEN":
                if byte > MAX_PAYLOAD:
                    self._reset()
                else:
                    self._len = byte
                    self._phase = "READ_SEQ"
            elif phase == "READ_SEQ":
                self._seq = byte
                self._phase = "READ_TYPE"
            elif phase == "READ_TYPE":
                self._type = byte
                self._payload = bytearray()
                self._phase = "READ_CRC_LO" if self._len == 0 else "READ_PAYLOAD"
            elif phase == "READ_PAYLOAD":
                self._payload.append(byte)
                if len(self._payload) >= self._len:
                    self._phase = "READ_CRC_LO"
            elif phase == "READ_CRC_LO":
                self._crc_lo = byte
                self._phase = "READ_CRC_HI"
            elif phase == "READ_CRC_HI":
                self._crc = self._crc_lo | (byte << 8)
                self._phase = "READ_END"
            elif phase == "READ_END":
                self._finish(byte, frames)
        return frames

    def _finish(self, end_byte: int, frames: list[Frame]) -> None:
        inner = bytes([self._seq, self._type]) + bytes(self._payload)
        if end_byte == FRAME_END and crc16_ccitt(inner) == self._crc:
            frames.append(Frame(self._seq, self._type, bytes(self._payload)))
        else:
            self.crc_errors += 1
        self._reset()


@dataclass
class Status:
    """Decoded RSP_STATUS payload (28 bytes)."""

    j1_steps: int = 0
    j2_steps: int = 0
    j3_steps: int = 0
    j4_us: int = 1500
    gripper_us: int = 1500
    fault_flags: int = 0
    state_bits: int = 0
    uptime_ms: int = 0
    mcu_temp_c10: int = 0
    reserved: int = 0

    _STRUCT = struct.Struct("<iiiHHHHIhH")

    def pack(self) -> bytes:
        return self._STRUCT.pack(
            self.j1_steps, self.j2_steps, self.j3_steps,
            self.j4_us, self.gripper_us, self.fault_flags,
            self.state_bits, self.uptime_ms, self.mcu_temp_c10,
            self.reserved,
        )

    @classmethod
    def unpack(cls, payload: bytes) -> "Status":
        return cls(*cls._STRUCT.unpack(payload))


@dataclass
class JointTargets:
    """CMD_SET_JOINT_TARGETS payload (18 bytes)."""

    j1_steps: int = 0
    j2_steps: int = 0
    j3_steps: int = 0
    j4_us: int = 1500
    gripper_us: int = 1500
    flags: int = 0

    _STRUCT = struct.Struct("<iiiHHH")

    def pack(self) -> bytes:
        return self._STRUCT.pack(
            self.j1_steps, self.j2_steps, self.j3_steps,
            self.j4_us, self.gripper_us, self.flags,
        )

    @classmethod
    def unpack(cls, payload: bytes) -> "JointTargets":
        return cls(*cls._STRUCT.unpack(payload))


@dataclass
class FaultEvent:
    """EVT_FAULT payload (4 bytes)."""

    new_bits: int = 0
    current_flags: int = 0

    _STRUCT = struct.Struct("<HH")

    def pack(self) -> bytes:
        return self._STRUCT.pack(self.new_bits, self.current_flags)

    @classmethod
    def unpack(cls, payload: bytes) -> "FaultEvent":
        return cls(*cls._STRUCT.unpack(payload))


@dataclass
class BootEvent:
    """EVT_BOOT payload (8 bytes)."""

    proto_major: int = 0
    proto_minor: int = 0
    eeprom_slot: int = 0
    recovered: int = 0
    git_short: int = 0

    _STRUCT = struct.Struct("<BBBBI")

    def pack(self) -> bytes:
        return self._STRUCT.pack(
            self.proto_major, self.proto_minor, self.eeprom_slot,
            self.recovered, self.git_short,
        )

    @classmethod
    def unpack(cls, payload: bytes) -> "BootEvent":
        return cls(*cls._STRUCT.unpack(payload))


@dataclass
class Ack:
    """RSP_ACK payload (2 bytes)."""

    acked_seq: int = 0
    acked_type: int = 0

    @classmethod
    def unpack(cls, payload: bytes) -> "Ack":
        return cls(payload[0], payload[1])


@dataclass
class Nak:
    """RSP_NAK payload (3 bytes)."""

    nak_seq: int = 0
    nak_type: int = 0
    err_code: int = 0

    @classmethod
    def unpack(cls, payload: bytes) -> "Nak":
        return cls(payload[0], payload[1], payload[2])
