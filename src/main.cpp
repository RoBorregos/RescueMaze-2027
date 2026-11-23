#include <Arduino.h>
#include "config.h"
#include "chassis.h"
#include "imu.h"
#include "movements.h"

// Global and never copied: the encoder interrupts keep pointers into the Chassis motors.
Chassis g_chassis;
Imu g_imu;
Movements g_movements;

// One tile forward, turn right, one tile forward, turn left. Positive degrees = clockwise (right).
struct Step {
    bool is_turn;
    float value;  // cm for advance, degrees for turn
};

static const Step kSequence[] = {
    {false, Geometry::kTileSizeCm}, //One tile forward
    {true, 90.0f}, //Turn right
    {false, Geometry::kTileSizeCm}, //One tile forward
    {true, -90.0f}, //Turn left
    {true, 180.0f}, //Turn around
};

static const uint8_t kSequenceLength = sizeof(kSequence) / sizeof(kSequence[0]);

static constexpr uint32_t kStartDelayMs = 3000;  // Time to put the robot down after reset
static constexpr uint32_t kPrintPeriodMs = 100;  // Telemetry over USB, 10 times per second

// Set to true once to spin M1..M4 one by one at startup and identify the wheels.
static constexpr bool kSpinMotorsAtStart = false;

static uint32_t last_loop_us = 0;
static uint32_t last_print_ms = 0;
static uint8_t step_index = 0;

// Spin motor M1 to M4 one at a time, 1 s each. ignoring kMotorInverted. Wheels in the air.
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
    Serial.print(",heading=");
    Serial.print(g_imu.getHeading());
    Serial.print(",target=");
    Serial.print(g_movements.getTargetHeading());
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

    if(kSpinMotorsAtStart) {
        for (uint8_t i = 0; i < MotorID::kCount; i++) spinMotor(i);
    }

    g_chassis.begin(); //Motor and PID setup for each wheel
    if (!g_imu.begin()) {
        Serial.println("IMU failed to initialize");
    }
    delay(kStartDelayMs);

    while (g_imu.isReady() == false && millis() < kStartDelayMs + 2000) { //Wait for the first heading
        g_imu.update();
    }
    g_movements.begin(&g_chassis, &g_imu); //Connect the Movements class to the Chassis class

    Serial.println(g_imu.isReady() ? "start" : "no heading, sequence disabled");
    last_loop_us = micros();
}

void loop() {
    uint32_t now_us = micros();
    if (now_us - last_loop_us < Control::kPeriodMs * 1000UL) return; //If the time since the last loop is less than the control period, return early.

    float dt = (now_us - last_loop_us) / 1000000.0f; //Time since last loop in seconds
    last_loop_us = now_us; 

    g_imu.update(); //Read the IMU, update the heading
    g_chassis.update(dt); //For each wheel, turn ticks to cm/s, update the PID and set the power to the motor
    g_movements.update(dt); //Checks if the current move is done, if not, checks if it has timed out, if not, checks how much distance is left and sets the target speed for each wheel accordingly.
    
    // Start the next step when the previous one is finished. Without a heading, turns could never finish, so nothing starts.
    if (g_imu.isReady() && !g_movements.isBusy() && step_index < kSequenceLength) {
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
