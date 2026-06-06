// SPDX-License-Identifier: MIT
//
// FORTIS Teensy 4.1 — servo sweep test sketch.
//
// Purpose: standalone smoke test for hobby-servo wiring + PWM on pin 29
// (the GRIPPER slot). NOT for the Hitec D845WP — its safe range is wider
// (500–2500 us); this sketch deliberately stays at the conservative
// gripper range to avoid driving a generic servo past its endstops.
//
// What it does:
//   1. Attaches a Servo on pin 29 at the gripper µs range
//   2. Moves to neutral (1500 µs) on boot, holds 1 second
//   3. Sweeps slowly between min and max, ~50 µs per step, 100 ms per step
//   4. Prints the current µs target to USB Serial every step (115200 baud)
//   5. Reverses at each end and keeps going forever
//
// Wiring (must match):
//   Signal (orange/yellow) ──→ Teensy pin 29
//   Servo +5V (red)        ──→ EXTERNAL 5V supply (+)
//   Servo GND (brown/black)──→ EXTERNAL 5V supply (-)
//                              │
//                              └──→ Teensy GND  (any GND pin)
//
// Safety:
//   - DO NOT power the servo from the Teensy USB. Use external 5V.
//   - Common ground between Teensy and the 5V supply is mandatory.
//   - If your servo is unhappy at 1000 or 2000 µs, narrow the range below
//     and re-flash.

#include <Arduino.h>
#include <Servo.h>

static const int      kServoPin   = 29;     // gripper slot per firmware pin map
// Conservative range for an unknown hobby gripper servo: 1000-2000 us is the
// industry-standard "safe" PWM window almost every hobby servo accepts. If
// the gripper takes this range cleanly, we'll widen it (500-2500 us) in a
// follow-up pass to find its actual mechanical endstops. Watch the bench
// supply current and stop the sweep if it spikes.
static const uint16_t kMinUs      = 1200;
static const uint16_t kMaxUs      = 2300;
static const uint16_t kNeutralUs  = 1500;
static const uint16_t kStepUs     = 50;     // travel per tick
static const uint32_t kStepDelayMs = 100;   // delay between ticks (slow)

static Servo g_servo;

void setup() {
    Serial.begin(1000000);   // match main firmware so the same Serial Monitor session works
    // Brief grace period so the host can attach Serial Monitor first
    delay(500);
    Serial.println(F("[servo_sweep] boot. Pin 29 (gripper), conservative range 1000-2000 us."));

    g_servo.attach(kServoPin, kMinUs, kMaxUs);
    g_servo.writeMicroseconds(kNeutralUs);
    Serial.print(F("[servo_sweep] neutral: ")); Serial.println(kNeutralUs);
    delay(1000);
}

void loop() {
    static uint16_t us  = kNeutralUs;
    static int16_t  dir = +1;

    us = (uint16_t)((int32_t)us + dir * (int32_t)kStepUs);

    if (us >= kMaxUs) { us = kMaxUs; dir = -1; }
    if (us <= kMinUs) { us = kMinUs; dir = +1; }

    g_servo.writeMicroseconds(us);
    Serial.print(F("us=")); Serial.println(us);

    delay(kStepDelayMs);
}
