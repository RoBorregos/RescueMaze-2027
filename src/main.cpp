#include <Arduino.h>
#include "config.h"
#include "chassis.h"
#include "movements.h"

// Global and never copied: the encoder interrupts keep pointers into the Chassis motors.
Chassis g_chassis;
Movements g_movements;

// One tile forward, turn right, one tile forward, turn left.
// Positive degrees = clockwise (right).
struct Step {
    bool is_turn;
    float value;  // cm for advance, degrees for turn
};
static const Step kSequence[] = {
    {false, Geometry::kTileSizeCm},
    {true, 90.0f},
    {false, Geometry::kTileSizeCm},
    {true, -90.0f},
};
static const uint8_t kSequenceLength = sizeof(kSequence) / sizeof(kSequence[0]);

static constexpr uint32_t kStartDelayMs = 3000;  // Time to put the robot down after reset
static constexpr uint32_t kPrintPeriodMs = 100;  // Telemetry over USB, 10 times per second

static uint32_t last_loop_us = 0;
static uint32_t last_print_ms = 0;
static uint8_t step_index = 0;

// TEMPORARY: spin motor M1..M4 one at a time, 1 s each. Wheels in the air.
static void spinMotor(uint8_t i) {
    Serial.print("Spinning M");
    Serial.println(i + 1);
    pinMode(Pins::kMotorIn1[i], OUTPUT);
    pinMode(Pins::kMotorIn2[i], OUTPUT);
    digitalWrite(Pins::kMotorIn1[i], HIGH);
    digitalWrite(Pins::kMotorIn2[i], LOW);
    analogWrite(Pins::kMotorPwm[i], 80);
    delay(1000);
    analogWrite(Pins::kMotorPwm[i], 0);
    delay(1000);
}

static void printTelemetry() {
    Serial.print(millis());
    Serial.print(",step=");
    Serial.print(step_index);
    Serial.print(",status=");
    Serial.print(static_cast<int>(g_movements.getStatus()));
    Serial.print(",dist_l=");
    Serial.print(g_chassis.getLeftDistance());
    Serial.print(",dist_r=");
    Serial.print(g_chassis.getRightDistance());
    Serial.print(",speed_l=");
    Serial.print(g_chassis.getLeftSpeed());
    Serial.print(",speed_r=");
    Serial.println(g_chassis.getRightSpeed());
}

void setup() {
    Serial.begin(Comms::kUsbBaudRate);
    for (uint8_t i = 0; i < 4; i++) spinMotor(i);

    g_chassis.begin();
    g_movements.begin(&g_chassis);

    delay(kStartDelayMs);
    Serial.println("start");
    last_loop_us = micros();
}

void loop() {
    uint32_t now_us = micros();
    if (now_us - last_loop_us < Control::kPeriodMs * 1000UL) return;

    float dt = (now_us - last_loop_us) / 1000000.0f;
    last_loop_us = now_us;

    g_chassis.update(dt);
    g_movements.update();
    
    // Start the next step when the previous one is finished.
    if (!g_movements.isBusy() && step_index < kSequenceLength) {
        if (g_movements.getStatus() == MoveStatus::kTimeout) {
            Serial.println("timeout, sequence stopped");
            step_index = kSequenceLength;
        } else {
            const Step& step = kSequence[step_index];
            if (step.is_turn) {
                g_movements.startTurn(step.value);
            } else {
                g_movements.startAdvance(step.value);
            }
            step_index++;
        }
    }

    if (millis() - last_print_ms >= kPrintPeriodMs) {
        last_print_ms = millis();
        printTelemetry();
    }
}
