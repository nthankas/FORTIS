// SPDX-License-Identifier: MIT
//
// FORTIS Teensy 4.1 — gripper hold-grip test sketch.
//
// Purpose: drive the gripper servo on pin 29 to a closed/"grip" position
// and hold it there indefinitely. Used to verify the gripper sustains a
// commanded position cleanly without the supply current creeping up
// (which would mean it's stalling, not holding).
//
// Sequence:
//   1. On boot, attach the servo and drive to kReleaseUs ("open"/release
//      position) so the gripper has a known starting point regardless of
//      where it was when powered on.
//   2. Every kCycleHoldMs (default 5 s), toggle between RELEASE and GRIP.
//   3. Log each transition on USB serial.
//   4. Loop forever.
//
// What "pass" looks like:
//   - Gripper visibly opens and closes on a 5-on / 5-off rhythm
//   - Supply current stays low (<200 mA) at both hold positions
//   - Serial Monitor shows alternating "-> GRIP at X us" / "-> RELEASE at Y us"
//
// What "fail" looks like:
//   - Loud whine / current >500 mA at one of the hold positions -> that
//     position is past a mechanical endstop. Pull that endpoint back toward
//     1500 us and re-flash.
//   - Light clicking at one position with low current is gear chatter from
//     the control loop and is normal for cheap servos with no external load.
//
// Wiring (must match main firmware pin map):
//   Signal (orange/yellow) ──→ Teensy pin 29
//   Servo +5V (red)        ──→ EXTERNAL 5V supply (+), ~1A limit is plenty
//   Servo GND (brown/black)──→ EXTERNAL 5V supply (-)
//                              │
//                              └──→ Teensy GND  (any GND pin)

#include <Arduino.h>
#include <Servo.h>

static const int      kPin           = 29;     // gripper slot per firmware pin map
static const uint16_t kMinUs         = 1200;   // matches characterized safe range
static const uint16_t kMaxUs         = 2500;
static const uint16_t kReleaseUs     = 2500;   // brief startup pose, also "open" if grip is at max
static const uint16_t kGripUs        = 1400;   // flip toward 1200 if 2000 turns out to be "release"

static const uint32_t kCycleHoldMs   = 5000;   // hold time at each position before toggling

static Servo g_servo;

void setup() {
    Serial.begin(1000000);   // match main firmware so one Serial Monitor session covers everything
    delay(500);
    Serial.println(F("[gripper_hold] boot. Pin 29, cycling release/grip every 5 s."));

    g_servo.attach(kPin, kMinUs, kMaxUs);
    g_servo.writeMicroseconds(kReleaseUs);
    Serial.print(F("[gripper_hold] start at release: ")); Serial.println(kReleaseUs);
}

void loop() {
    static bool s_gripped = false;
    static uint32_t s_last_switch = 0;
    const uint32_t now = millis();

    if ((now - s_last_switch) >= kCycleHoldMs) {
        s_last_switch = now;
        s_gripped = !s_gripped;
        const uint16_t target = s_gripped ? kGripUs : kReleaseUs;
        g_servo.writeMicroseconds(target);
        Serial.print(F("[gripper_hold] -> "));
        Serial.print(s_gripped ? F("GRIP at ") : F("RELEASE at "));
        Serial.print(target); Serial.println(F(" us"));
    }
}
