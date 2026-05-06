# Teensy Firmware — Handoff Guide

Hi Cesar — this is your patch. The skeleton, protocol spec, and a desktop mock are
in place but **nothing has been compiled or run on hardware yet**. Below is the
current state, what's safe to change vs. what needs coordination, and what's left.

Nikhil + Claude handle the rest of the stack (ROS 2 nodes, Jetson side, sim,
URDF). You own the firmware, the protocol document, and the mock.

---

## 1. What's in your scope

| File | Purpose |
|---|---|
| `firmware/teensy/main.ino` | Firmware skeleton. TeensyStep for J1/J2/J3, hardware PWM for J4 + gripper, EEPROM position recovery, serial framing. |
| `firmware/teensy/PROTOCOL.md` | Wire-format spec for the host↔Teensy serial link. Treat as a contract — see §5. |
| `tools/mock_teensy.py` | Pure-Python pty simulator of the Teensy side. Lets the Jetson team develop without the board. |
| `firmware/teensy/HANDOFF.md` | This file. |

## 2. What's NOT yours (don't edit)

- `src/**` — ROS 2 packages (Nikhil + Claude)
- `sim/**` — Gazebo / Mujoco models
- `urdf/**` — robot description
- The future Jetson-side serial bridge node (will live under `src/`) — not yet written

If you find a bug that crosses the boundary (e.g., the host driver sends a
malformed frame), flag it; don't fix it yourself.

## 3. Hardware assumptions baked into the firmware

- **MCU:** Teensy 4.1 (iMXRT1062, 600 MHz, 3.3 V logic)
- **J1, J2, J3:** NEMA-23 closed-loop steppers via **CL57T-V41** drivers (step/dir/ena, optoisolated 5 V inputs)
- **Level shifters:** **SN74HCT245N** — one for outbound STEP/DIR/ENA (A→B, 3.3 V→5 V), one for inbound ALM (B→A, 5 V→3.3 V)
- **J4:** Hitec D845WP servo, hardware PWM @ 50 Hz, 500–2500 µs
- **Gripper:** generic hobby servo, hardware PWM @ 50 Hz, 1000–2000 µs
- **Host link:** USB CDC serial, 1 000 000 baud
- **Position recovery:** Teensy 4.1 EEPROM emulation, ping-pong slots with CRC

If the hardware spec changes (different driver, different servo, different shifter),
update PROTOCOL.md §1 and the `#define` block at the top of `main.ino` together.

## 4. Pin map (current)

| Function | Teensy pin |
|---|---:|
| J1 STEP / DIR / ALM | 2 / 3 / 22 |
| J2 STEP / DIR / ALM | 4 / 5 / 23 |
| J3 STEP / DIR / ALM | 6 / 7 / 20 |
| Shared driver ENABLE (active-low) | 9 |
| J4 servo (FlexPWM3.1.B) | 28 |
| Gripper servo (FlexPWM3.1.A) | 29 |
| SN74HCT245N /OE / DIR | 14 / 15 |
| External E-STOP (active-low, pull-up) | 21 |
| Status LED | 13 (built-in) |

Pin map appears in two places: `main.ino` (the `#define` block ~lines 33–61) and
PROTOCOL.md §1. **Keep them in sync** — if you change a pin, change both.

## 5. Don't touch without coordination — protocol contract

These define the wire format. Changing them silently will break the Jetson side
the moment it's written.

- **Frame layout** (start byte 0xA5, LEN, SEQ, TYPE, payload, CRC-16-CCITT, end byte 0x5A)
- **CRC polynomial 0x1021, init 0xFFFF**, computed over `[SEQ, TYPE, PAYLOAD]`
- **Numeric values** of `CMD_*`, `RSP_*`, `EVT_*`, `FAULT_*`, `ERR_*` constants
- **Payload byte layouts** documented in PROTOCOL.md §3
- **Watchdog timeout** (250 ms) and **heartbeat cadence** expectation (≤100 ms)

**OK to extend without asking** — append new message types at the next available
ID. Send me a Slack msg or open a PR comment so I can mirror it on the host side.

**Need to change layout?** Bump `PROTO_MINOR` (or `PROTO_MAJOR` if breaking) in
`main.ino` and PROTOCOL.md, and ping Nikhil before merging.

## 6. Safe to change freely

- Internal firmware structure — state machines, helper functions, timing constants
- Config block (steps/rev, microstepping, max speed/accel) — currently 200·8, 8000 sps, 20000 sps²
- TODO implementations (see §8)
- `mock_teensy.py` internals, as long as the round-trip behavior in §7 still passes
- Pin assignments **if** PROTOCOL.md §1 is updated in the same commit

## 7. Current verification status

| Check | Result |
|---|---|
| `python -m py_compile tools/mock_teensy.py` | ✅ pass |
| `mock_teensy.py --help` | ✅ pass |
| Constant consistency: `main.ino` ↔ `PROTOCOL.md` ↔ `mock_teensy.py` | ✅ all 33 constants align |
| Frame start/end/CRC params match across all three | ✅ |
| Mock round-trip: heartbeat ACK, GET_STATUS, SET_TARGETS, heartbeat-timeout fault | ✅ all 4 cases pass |
| `arduino-cli compile --fqbn teensy:avr:teensy41 main.ino` | ❌ **BLOCKED** — see §8 |
| Bench bring-up (real Teensy + drivers + servos) | ❌ not started |

## 8. Known blockers

### 8.1 TeensyStep doesn't compile against the current Teensy core

This is upstream library breakage, not anything wrong with `main.ino`.

- **TeensyStep 2.3.4** (the only version in the Arduino library registry) gates
  its Teensy 4 implementation on `__IMXRT1052__`
- **Teensy core 1.60.0** defines `__IMXRT1062__` (the actual MIMXRT1062 chip)
- TeensyStep also calls `dwt_getCycles()`, which the new core no longer provides

Result: `#include <TeensyStep.h>` itself fails to compile. Setting `MOCK_MODE=1`
does NOT help — the include is unconditional in the current `main.ino`.

**Three options, in order of recommendation:**

1. **Vendor a patched TeensyStep into `firmware/teensy/lib/TeensyStep/`.**
   The fix on the upstream `next` branch (or community forks like `tni/TeensyStep`)
   has the iMXRT1062 + cycle-counter fixes. Pin a known-good commit. Lowest risk
   to the rest of the codebase.
2. **Pin the Teensy core to ≤1.57.x.** Quick, but blocks any other Teensy 4.1
   work that benefits from newer core features. Not a long-term fix.
3. **Switch to AccelStepper or TMCStepper.** Lots of churn — would mean rewriting
   the J1/J2/J3 motion code in `main.ino`. Only do this if option 1 turns out
   to be a maintenance burden.

Whichever path you pick: also wrap `#include <TeensyStep.h>` and the
`Stepper`/`StepControl` declarations in `#if !MOCK_MODE` so we can at least exercise
the protocol/serial path with `MOCK_MODE=1` from a desktop without the library.

### 8.2 Mock watchdog arms at startup

`tools/mock_teensy.py` starts its 250 ms heartbeat watchdog inside
`MockState.__init__`. A host that takes >250 ms to attach sees a spurious
`EVT_FAULT(FAULT_HEARTBEAT_TIMEOUT)` arrive before its first command. Cosmetic
in normal operation, annoying for tests.

**Fix:** defer arming the watchdog until the first inbound CMD is received.
Trivial change; do this when you're in the file for something else.

## 9. TODOs flagged in main.ino (search `// TODO`)

| Symbol / location | What's missing |
|---|---|
| `handleHomeRequest` | Stubbed, NAKs with `ERR_NOT_IMPLEMENTED`. Needs per-joint move-until-ALM-or-limit-switch + back-off + `setPosition(0)`. |
| `readMcuTempC10` | Returns 0. Wire to `tempmonGetTemp() * 10.0f` once Teensyduino core version is pinned. |
| `CMD_SET_JOINT_VELOCITIES` | Currently uses `abs()` for `setMaxSpeed`. Pure-velocity (continuous run) mode is not implemented — TeensyStep's `RotateControl` is the right primitive. |
| `clamp_to_limits` flag in `CMD_SET_JOINT_TARGETS` | Parsed but ignored. Joint software limits aren't defined yet. Define limits per joint (likely matches whatever Nikhil documents in URDF — ask before guessing). |
| `PIN_DRV_ENABLE` polarity | Whether the CL57T-V41 ENA optocoupler is common-anode or common-cathode determines the polarity. **Verify on the bench with a multimeter before powering motors.** |

## 10. What's left to test / integrate

In rough order — don't start later items before earlier ones unless you have a
reason.

1. **Resolve §8.1** so `arduino-cli compile` succeeds.
2. **Compile + upload to a real Teensy 4.1.** Confirm the LED blinks (heartbeat indicator) and that USB serial enumerates at 1 Mbaud.
3. **Scope STEP/DIR signals at the SN74HCT245N B-side outputs** before connecting drivers. Verify clean 5 V edges at the expected pin map. Catches level-shifter wiring bugs without smoking a driver.
4. **Wire one driver (J1).** Verify ENA polarity with a multimeter (see §9). Move the joint slowly with a `CMD_SET_JOINT_TARGETS` from the host (use the mock's host-side test client as a starting point).
5. **Force a driver fault** (e.g., disconnect a motor phase) and verify `FAULT_DRIVER_ALARM_J1` is raised in `RSP_STATUS` and an `EVT_FAULT` is emitted.
6. **Repeat for J2, J3.**
7. **Servo bring-up: J4 then gripper.** Scope the PWM first, then attach the servo.
8. **EEPROM round-trip:** command motion, kill power mid-move, power back on, verify `RSP_STATUS` reports the last-checkpointed positions (not zero, not garbage). Test the ping-pong slot logic by interrupting writes with a quick power-cycle during a checkpoint.
9. **E-stop loop:** verify the active-low input on pin 21 raises `FAULT_E_STOP`, latches motors disabled, and clears via `CMD_CLEAR_FAULTS` only after the input goes high again.
10. **Heartbeat watchdog:** stop sending heartbeats from the host, verify motors disable within ~250 ms and the fault is reported.
11. **Integration with the Jetson-side bridge** (Nikhil + Claude will write this once 1–10 are green). Plan: a `serial_bridge` ROS 2 node that exposes the protocol as joint-state + joint-trajectory topics. We'll coordinate the API at that point.

## 11. Working locally

- **Edit / build:** Arduino IDE 2.x with Teensyduino, OR `arduino-cli` once §8.1 is unblocked. Everything should compile from `firmware/teensy/main.ino` standalone.
- **Protocol-only work without a Teensy:** run `python tools/mock_teensy.py --verbose`, point a host script at the slave pty path it prints. The mock supports `--inject-fault` for testing host-side fault handling.
- **Branch:** all of this lives on `feat/teensy-protocol`. Make commits there or branch off it. **Don't merge to main until §10 steps 1–3 at minimum are green.**

## 12. Quick contact points

- Protocol questions / extending message types → ping Nikhil; he'll mirror on the host side.
- ROS 2 / Jetson-side concerns → Nikhil + Claude.
- Hardware wiring questions (driver settings, shifter polarity, servo specs) → that's your call but flag anything that diverges from §3 in PROTOCOL.md.
