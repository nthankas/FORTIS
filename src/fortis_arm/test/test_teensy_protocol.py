"""
Unit tests for fortis_arm.teensy_protocol.

Pure-stdlib: no rclpy, no serial. Covers CRC vectors derived by hand
from tools/mock_teensy.py, frame round-trips, parser resync and bad-CRC
handling, and payload pack/unpack.
"""

from __future__ import annotations

from fortis_arm.teensy_protocol import (
    Fault,
    FrameParser,
    JointTargets,
    MsgType,
    Status,
    crc16_ccitt,
    encode,
    fault_names,
)


def test_crc_known_vectors():
    # Derived from the mock crc16_ccitt (poly 0x1021, init 0xFFFF).
    assert crc16_ccitt(b"123456789") == 0x29B1
    assert crc16_ccitt(b"") == 0xFFFF
    # Heartbeat inner bytes [seq=0, type=CMD_HEARTBEAT=0x01].
    assert crc16_ccitt(bytes([0x00, 0x01])) == 0x0D2E


def test_encode_heartbeat_frame_bytes():
    # A5 | LEN=0 | SEQ=0 | TYPE=0x01 | CRC_LO=0x2E | CRC_HI=0x0D | 0x5A
    assert encode(0, int(MsgType.CMD_HEARTBEAT)) == bytes(
        [0xA5, 0x00, 0x00, 0x01, 0x2E, 0x0D, 0x5A]
    )


def test_frame_roundtrip_targets():
    payload = JointTargets(
        j1_steps=100, j2_steps=-50, j3_steps=0,
        j4_us=1500, gripper_us=1500, flags=1,
    ).pack()
    frame = encode(42, int(MsgType.CMD_SET_JOINT_TARGETS), payload)
    parser = FrameParser()
    out = parser.feed(frame)
    assert len(out) == 1
    assert out[0].seq == 42
    assert out[0].msg_type == int(MsgType.CMD_SET_JOINT_TARGETS)
    assert JointTargets.unpack(out[0].payload).j2_steps == -50


def test_parser_handles_split_feeds():
    frame = encode(9, int(MsgType.RSP_STATUS), Status(j1_steps=1).pack())
    parser = FrameParser()
    out = []
    for byte in frame:
        out += parser.feed(bytes([byte]))
    assert len(out) == 1
    assert out[0].seq == 9


def test_parser_resync_after_garbage():
    parser = FrameParser()
    frame = encode(5, int(MsgType.CMD_HEARTBEAT))
    out = parser.feed(b"\x00\xff\x11garbage" + frame)
    assert [f.seq for f in out] == [5]
    assert parser.crc_errors == 0


def test_parser_recovers_from_false_start():
    parser = FrameParser()
    good = encode(7, int(MsgType.CMD_GET_STATUS))
    # 0xA5 then LEN=0xFF (>240) is rejected; parser resets and recovers.
    out = parser.feed(bytes([0xA5, 0xFF]) + good)
    assert [f.seq for f in out] == [7]


def test_parser_drops_bad_crc():
    parser = FrameParser()
    frame = bytearray(
        encode(3, int(MsgType.CMD_SET_JOINT_TARGETS),
               JointTargets(j1_steps=10).pack())
    )
    # Corrupt one payload byte; CRC then mismatches at the END byte.
    frame[4] ^= 0xFF
    good = encode(4, int(MsgType.CMD_HEARTBEAT))
    out = parser.feed(bytes(frame) + good)
    assert parser.crc_errors == 1
    assert [f.seq for f in out] == [4]


def test_status_pack_unpack_roundtrip():
    st = Status(
        j1_steps=-5, j2_steps=1000, j3_steps=7,
        j4_us=1600, gripper_us=1200, fault_flags=0x0002,
        state_bits=0x0001, uptime_ms=123456, mcu_temp_c10=305, reserved=0,
    )
    assert len(st.pack()) == 28
    assert Status.unpack(st.pack()) == st


def test_joint_targets_pack_len():
    assert len(JointTargets().pack()) == 18


def test_fault_names_decode():
    flags = int(Fault.HEARTBEAT_TIMEOUT) | int(Fault.DRIVER_ALARM_J1)
    names = fault_names(flags)
    assert "HEARTBEAT_TIMEOUT" in names
    assert "DRIVER_ALARM_J1" in names
    assert "DRIVER_ALARM_J2" not in names
