#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Pseudo-terminal-backed mock of the FORTIS Teensy 4.1 motion controller.

This script simulates the Teensy side of the binary protocol defined in
``firmware/teensy/PROTOCOL.md``. It opens a Unix pseudo-terminal (pty) pair,
prints the slave-side path on startup, and then runs a tiny event loop that:

* parses incoming frames (start/len/seq/type/payload/CRC/end)
* responds to ``CMD_GET_STATUS`` with a synthetic ``RSP_STATUS``
* ACKs other commands (NAKs malformed ones)
* integrates joint motion at constant velocity between status polls so that
  ``RSP_STATUS`` reflects "moving" steppers
* tracks the heartbeat watchdog and emits ``EVT_FAULT`` after a 250 ms timeout
* can inject a synthetic fault on demand for host-side test coverage

How to use
----------
::

    $ python tools/mock_teensy.py --verbose
    [mock_teensy] pty slave: /dev/pts/7
    ...

Then point a host program at ``/dev/pts/7`` (or whatever path is printed) at
1 000 000 baud. A pty is byte-faithful, so the ``--baud`` flag below is
purely informational.

This script intentionally has no third-party dependencies; it sticks to the
Python standard library (``pty``, ``os``, ``select``, ``struct``, ``time``,
``argparse``). Targets Python 3.10+.
"""

from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import time
from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

# ``pty`` (and its ``termios`` dependency) is POSIX-only. Defer the import so
# that ``--help`` still works on Windows for protocol-doc cross-checks.
try:
    import pty as _pty
except ImportError:  # pragma: no cover - exercised on Windows
    _pty = None  # type: ignore[assignment]

# ---------------------------------------------------------------------------
# Protocol constants (must match firmware/teensy/PROTOCOL.md)
# ---------------------------------------------------------------------------

FRAME_START = 0xA5
FRAME_END = 0x5A
MAX_PAYLOAD = 240
HEARTBEAT_TIMEOUT_S = 0.250
FRAME_INTER_BYTE_TIMEOUT_S = 0.050


class MsgType(IntEnum):
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
    ENABLED = 1 << 0
    MOVING = 1 << 1
    HOMED_J1 = 1 << 2
    HOMED_J2 = 1 << 3
    HOMED_J3 = 1 << 4


# ---------------------------------------------------------------------------
# CRC-16-CCITT (poly 0x1021, init 0xFFFF, no reflect, no final XOR)
# ---------------------------------------------------------------------------


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# Frame I/O
# ---------------------------------------------------------------------------


def build_frame(msg_type: int, payload: bytes, seq: int) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD}")
    inner = bytes([seq & 0xFF, msg_type & 0xFF]) + payload
    crc = crc16_ccitt(inner)
    return (
        bytes([FRAME_START, len(payload), seq & 0xFF, msg_type & 0xFF])
        + payload
        + bytes([crc & 0xFF, (crc >> 8) & 0xFF, FRAME_END])
    )


@dataclass
class ParsedFrame:
    seq: int
    msg_type: int
    payload: bytes


class FrameParser:
    """Tiny streaming parser that mirrors the firmware state machine."""

    def __init__(self) -> None:
        self._reset()

    def _reset(self) -> None:
        self._phase = "WAIT_START"
        self._len = 0
        self._seq = 0
        self._type = 0
        self._payload = bytearray()
        self._crc_lo = 0
        self._crc = 0
        self._started = 0.0

    def feed(self, data: bytes) -> tuple[list[ParsedFrame], list[ErrCode]]:
        """Consume bytes; return any complete frames and any per-frame errors."""
        frames: list[ParsedFrame] = []
        errors: list[ErrCode] = []
        now = time.monotonic()
        if self._phase != "WAIT_START" and (now - self._started) > FRAME_INTER_BYTE_TIMEOUT_S:
            self._reset()

        for b in data:
            phase = self._phase
            if phase == "WAIT_START":
                if b == FRAME_START:
                    self._phase = "READ_LEN"
                    self._started = now
            elif phase == "READ_LEN":
                if b > MAX_PAYLOAD:
                    errors.append(ErrCode.LENGTH)
                    self._reset()
                else:
                    self._len = b
                    self._phase = "READ_SEQ"
            elif phase == "READ_SEQ":
                self._seq = b
                self._phase = "READ_TYPE"
            elif phase == "READ_TYPE":
                self._type = b
                self._payload = bytearray()
                self._phase = "READ_CRC_LO" if self._len == 0 else "READ_PAYLOAD"
            elif phase == "READ_PAYLOAD":
                self._payload.append(b)
                if len(self._payload) >= self._len:
                    self._phase = "READ_CRC_LO"
            elif phase == "READ_CRC_LO":
                self._crc_lo = b
                self._phase = "READ_CRC_HI"
            elif phase == "READ_CRC_HI":
                self._crc = self._crc_lo | (b << 8)
                self._phase = "READ_END"
            elif phase == "READ_END":
                if b != FRAME_END:
                    errors.append(ErrCode.CRC)
                    self._reset()
                    continue
                inner = bytes([self._seq, self._type]) + bytes(self._payload)
                if crc16_ccitt(inner) != self._crc:
                    errors.append(ErrCode.CRC)
                else:
                    frames.append(ParsedFrame(self._seq, self._type, bytes(self._payload)))
                self._reset()
        return frames, errors


# ---------------------------------------------------------------------------
# Mock device state
# ---------------------------------------------------------------------------


@dataclass
class MockState:
    j1_steps: int = 0
    j2_steps: int = 0
    j3_steps: int = 0
    j1_target: int = 0
    j2_target: int = 0
    j3_target: int = 0
    j1_vel: int = 4000  # steps/sec
    j2_vel: int = 4000
    j3_vel: int = 4000
    j4_us: int = 1500
    gripper_us: int = 1500
    fault_flags: int = 0
    state_flags: int = (
        int(StateBit.ENABLED) | int(StateBit.HOMED_J1) | int(StateBit.HOMED_J2)
        | int(StateBit.HOMED_J3)
    )
    last_heartbeat: float = field(default_factory=time.monotonic)
    last_motion_update: float = field(default_factory=time.monotonic)
    boot_time: float = field(default_factory=time.monotonic)

    def integrate(self, now: float) -> None:
        dt = now - self.last_motion_update
        self.last_motion_update = now
        if dt <= 0:
            return
        moving = False
        for cur_attr, tgt_attr, vel in (
            ("j1_steps", "j1_target", self.j1_vel),
            ("j2_steps", "j2_target", self.j2_vel),
            ("j3_steps", "j3_target", self.j3_vel),
        ):
            cur = getattr(self, cur_attr)
            tgt = getattr(self, tgt_attr)
            if cur == tgt:
                continue
            step = int(abs(vel) * dt)
            if step <= 0:
                continue
            if tgt > cur:
                cur = min(tgt, cur + step)
            else:
                cur = max(tgt, cur - step)
            setattr(self, cur_attr, cur)
            if cur != tgt:
                moving = True
        if moving:
            self.state_flags |= int(StateBit.MOVING)
        else:
            self.state_flags &= ~int(StateBit.MOVING)


# ---------------------------------------------------------------------------
# Mock device
# ---------------------------------------------------------------------------


class MockTeensy:
    def __init__(
        self,
        master_fd: int,
        verbose: bool,
        inject_fault: str,
    ) -> None:
        self._fd = master_fd
        self._verbose = verbose
        self._parser = FrameParser()
        self._tx_seq = 0
        self._state = MockState()
        self._inject_fault = inject_fault
        self._inject_at = time.monotonic() + 1.0 if inject_fault != "none" else None

    # ---- helpers ----

    def _log(self, msg: str) -> None:
        if self._verbose:
            sys.stderr.write(f"[mock_teensy] {msg}\n")
            sys.stderr.flush()

    def _next_seq(self) -> int:
        s = self._tx_seq
        self._tx_seq = (self._tx_seq + 1) & 0xFF
        return s

    def _send(self, msg_type: int, payload: bytes) -> None:
        frame = build_frame(msg_type, payload, self._next_seq())
        try:
            os.write(self._fd, frame)
        except OSError as exc:
            self._log(f"write failed: {exc}")
            return
        self._log(f"TX type=0x{msg_type:02X} len={len(payload)}")

    def _ack(self, seq: int, msg_type: int) -> None:
        self._send(int(MsgType.RSP_ACK), bytes([seq, msg_type]))

    def _nak(self, seq: int, msg_type: int, err: ErrCode) -> None:
        self._send(int(MsgType.RSP_NAK), bytes([seq, msg_type, int(err)]))

    def _emit_fault(self, new_bits: int) -> None:
        prev = self._state.fault_flags
        self._state.fault_flags |= new_bits
        rising = self._state.fault_flags & ~prev
        if rising == 0:
            return
        self._state.state_flags &= ~int(StateBit.ENABLED)
        self._state.state_flags &= ~int(StateBit.MOVING)
        self._send(
            int(MsgType.EVT_FAULT),
            struct.pack("<HH", rising, self._state.fault_flags),
        )

    # ---- handlers ----

    def _handle(self, frame: ParsedFrame) -> None:
        t = frame.msg_type
        p = frame.payload
        seq = frame.seq
        self._state.last_heartbeat = time.monotonic()
        self._log(f"RX seq={seq} type=0x{t:02X} len={len(p)}")

        if t == int(MsgType.CMD_HEARTBEAT):
            if len(p) != 0:
                return self._nak(seq, t, ErrCode.LENGTH)
            self._ack(seq, t)
        elif t == int(MsgType.CMD_GET_STATUS):
            if len(p) != 0:
                return self._nak(seq, t, ErrCode.LENGTH)
            self._send_status()
        elif t == int(MsgType.CMD_ENABLE):
            if len(p) != 0:
                return self._nak(seq, t, ErrCode.LENGTH)
            if self._state.fault_flags:
                return self._nak(seq, t, ErrCode.FAULT_ACTIVE)
            self._state.state_flags |= int(StateBit.ENABLED)
            self._ack(seq, t)
        elif t == int(MsgType.CMD_DISABLE):
            if len(p) != 0:
                return self._nak(seq, t, ErrCode.LENGTH)
            self._state.state_flags &= ~int(StateBit.ENABLED)
            self._state.state_flags &= ~int(StateBit.MOVING)
            self._ack(seq, t)
        elif t == int(MsgType.CMD_SET_JOINT_TARGETS):
            if len(p) != 18:
                return self._nak(seq, t, ErrCode.LENGTH)
            j1, j2, j3, j4, gr, flags = struct.unpack("<iiiHHH", p)
            if not (500 <= j4 <= 2500) or not (1000 <= gr <= 2000):
                return self._nak(seq, t, ErrCode.BAD_PARAMETER)
            self._state.j1_target = j1
            self._state.j2_target = j2
            self._state.j3_target = j3
            self._state.j4_us = j4
            self._state.gripper_us = gr
            _ = flags
            self._ack(seq, t)
        elif t == int(MsgType.CMD_SET_JOINT_VELOCITIES):
            if len(p) != 12:
                return self._nak(seq, t, ErrCode.LENGTH)
            v1, v2, v3 = struct.unpack("<iii", p)
            self._state.j1_vel = v1 if v1 != 0 else 1
            self._state.j2_vel = v2 if v2 != 0 else 1
            self._state.j3_vel = v3 if v3 != 0 else 1
            self._ack(seq, t)
        elif t == int(MsgType.CMD_HOME_REQUEST):
            if len(p) != 1:
                return self._nak(seq, t, ErrCode.LENGTH)
            # Mock just claims homing succeeded so host code can exercise its
            # post-home path. Real firmware NAKs with NOT_IMPLEMENTED.
            mask = p[0]
            if mask & 0x01:
                self._state.j1_steps = 0
                self._state.j1_target = 0
                self._state.state_flags |= int(StateBit.HOMED_J1)
            if mask & 0x02:
                self._state.j2_steps = 0
                self._state.j2_target = 0
                self._state.state_flags |= int(StateBit.HOMED_J2)
            if mask & 0x04:
                self._state.j3_steps = 0
                self._state.j3_target = 0
                self._state.state_flags |= int(StateBit.HOMED_J3)
            self._ack(seq, t)
        elif t == int(MsgType.CMD_CLEAR_FAULTS):
            if len(p) != 2:
                return self._nak(seq, t, ErrCode.LENGTH)
            (mask,) = struct.unpack("<H", p)
            self._state.fault_flags &= ~mask & 0xFFFF
            self._ack(seq, t)
        else:
            self._nak(seq, t, ErrCode.UNKNOWN_TYPE)

    # ---- status ----

    def _send_status(self) -> None:
        s = self._state
        uptime_ms = int((time.monotonic() - s.boot_time) * 1000) & 0xFFFFFFFF
        payload = struct.pack(
            "<iiiHHHHIhH",
            s.j1_steps,
            s.j2_steps,
            s.j3_steps,
            s.j4_us,
            s.gripper_us,
            s.fault_flags & 0xFFFF,
            s.state_flags & 0xFFFF,
            uptime_ms,
            250,  # 25.0 C, fake
            0,    # reserved
        )
        assert len(payload) == 28, f"RSP_STATUS payload wrong: {len(payload)}"
        self._send(int(MsgType.RSP_STATUS), payload)

    # ---- main loop ----

    def boot(self) -> None:
        # EVT_BOOT: proto_major=1, proto_minor=0, slot=0, recovered=1, build=0
        self._send(int(MsgType.EVT_BOOT), struct.pack("<BBBBI", 1, 0, 0, 1, 0))

    def tick(self, now: float) -> None:
        self._state.integrate(now)

        if (now - self._state.last_heartbeat) > HEARTBEAT_TIMEOUT_S:
            if not (self._state.fault_flags & int(Fault.HEARTBEAT_TIMEOUT)):
                self._log("heartbeat timeout -> EVT_FAULT")
                self._emit_fault(int(Fault.HEARTBEAT_TIMEOUT))

        if self._inject_at is not None and now >= self._inject_at:
            mapping = {
                "heartbeat": int(Fault.HEARTBEAT_TIMEOUT),
                "driver_alm": int(Fault.DRIVER_ALARM_J1),
                "eeprom_crc": int(Fault.EEPROM_CRC),
            }
            bits = mapping.get(self._inject_fault, 0)
            if bits:
                self._log(f"injecting fault: {self._inject_fault}")
                self._emit_fault(bits)
            self._inject_at = None

    def feed(self, data: bytes) -> None:
        frames, errors = self._parser.feed(data)
        for err in errors:
            self._log(f"parse error: {err.name}")
        for f in frames:
            self._handle(f)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="mock_teensy",
        description="Pseudo-terminal mock of the FORTIS Teensy 4.1 firmware "
        "(see firmware/teensy/PROTOCOL.md).",
    )
    p.add_argument(
        "--baud",
        type=int,
        default=1_000_000,
        help="Informational only — pty is byte-faithful and does not honour baud.",
    )
    p.add_argument("--verbose", "-v", action="store_true", help="Log RX/TX to stderr.")
    p.add_argument(
        "--inject-fault",
        choices=["none", "heartbeat", "driver_alm", "eeprom_crc"],
        default="none",
        help="Inject a synthetic fault ~1 s after startup for host-side testing.",
    )
    p.add_argument(
        "--tick-hz",
        type=float,
        default=200.0,
        help="Internal motion/fault scan rate (Hz).",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    if _pty is None or not hasattr(_pty, "openpty"):
        sys.stderr.write("mock_teensy requires a Unix pty (POSIX only).\n")
        return 2

    master_fd, slave_fd = _pty.openpty()
    slave_path = os.ttyname(slave_fd)
    sys.stdout.write(f"[mock_teensy] pty slave: {slave_path}\n")
    sys.stdout.write(f"[mock_teensy] baud (informational): {args.baud}\n")
    sys.stdout.flush()

    # Make master non-blocking
    os.set_blocking(master_fd, False)

    device = MockTeensy(master_fd, verbose=args.verbose, inject_fault=args.inject_fault)
    device.boot()

    tick_period = 1.0 / max(args.tick_hz, 1.0)
    next_tick = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            timeout = max(0.0, next_tick - now)
            try:
                rlist, _, _ = select.select([master_fd], [], [], timeout)
            except InterruptedError:
                continue
            if rlist:
                try:
                    chunk = os.read(master_fd, 4096)
                except BlockingIOError:
                    chunk = b""
                except OSError as exc:
                    sys.stderr.write(f"[mock_teensy] read error: {exc}\n")
                    break
                if chunk:
                    device.feed(chunk)
            now = time.monotonic()
            if now >= next_tick:
                device.tick(now)
                next_tick = now + tick_period
    except KeyboardInterrupt:
        sys.stderr.write("\n[mock_teensy] interrupted\n")
    finally:
        try:
            os.close(master_fd)
        except OSError:
            pass
        try:
            os.close(slave_fd)
        except OSError:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
