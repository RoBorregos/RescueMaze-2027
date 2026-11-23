#include <Arduino.h>
#include <stdint.h>
#include "config.h"
#include "chassis.h"
#include "imu.h"
#include "movements.h"

// Global and never copied: the encoder interrupts keep pointers into the Chassis motors.
Chassis g_chassis;
Imu g_imu;
Movements g_movements;

static constexpr uint32_t kImuWaitMs = 2000;     // Max wait for the first heading at startup
static constexpr uint32_t kPrintPeriodMs = 100;  // Telemetry over USB, 10 times per second

static uint32_t last_loop_us = 0;
static uint32_t last_print_ms = 0;
static MoveStatus last_status = MoveStatus::kIdle;
static uint8_t step_index = 0;

// Set to true once to spin M1..M4 one by one at startup and identify the wheels.
static constexpr bool kSpinMotorsAtStart = false;

// Text typed in the serial monitor, until Enter is pressed.
static char command_buffer[32];
static uint8_t command_length = 0;

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

static void printHelp() {
    Serial.println("Commands (type and press Enter):");
    Serial.println("  a <cm>   advance, negative = backward   e.g. a 28");
    Serial.println("  t <deg>  turn, + right / - left         e.g. t 90");
    Serial.println("  p        pause the current move");
    Serial.println("  r        resume the paused move");
    Serial.println("  s        stop and cancel the move");
    Serial.println("  i        print the current state");
}

static void printTelemetry() {
    Serial.print(millis());
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

// Runs one typed line
static void runCommand(const char* line) {
    while (*line == ' ') line++;          // Skip spaces at the start
    char command = line[0];
    float value = atof(line + 1);         // The number after the letter (0 if none)

    bool is_move = (command == 'a' || command == 't');
    if (is_move && !g_imu.isReady()) {
        Serial.println("no heading from the BNO085, cannot move");
        return;
    }
    if (is_move && g_movements.isBusy()) {
        Serial.println("busy: wait for 'done' or send s");
        return;
    }

    switch (command) {
        case 'a':
            g_movements.startAdvance(value);
            Serial.print("advance ");
            Serial.print(value);
            Serial.println(" cm");
            break;
        case 't':
            g_movements.startTurn(value);
            Serial.print("turn ");
            Serial.print(value);
            Serial.println(" deg");
            break;
        case 'p':
            Serial.println(g_movements.pause() ? "paused" : "nothing to pause");
            break;
        case 'r':
            Serial.println(g_movements.resume() ? "resumed" : "nothing to resume");
            break;
        case 's':
            g_movements.cancel();
            Serial.println("stopped");
            break;
        case 'i':
            printTelemetry();
            break;
        default:
            printHelp();
            break;
    }
}

// Collects the characters typed in the serial monitor. Only reads what has already arrived, the control loop keeps 10 ms.
static void readSerialCommands() {
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            if (command_length > 0) {
                command_buffer[command_length] = '\0';
                runCommand(command_buffer);
                command_length = 0;
            }
        } else if (command_length < sizeof(command_buffer) - 1) {
            command_buffer[command_length] = c;
            command_length++;
        }
    }
}

// Prints a line when a move finishes or is aborted.
static void reportStatusChange() {
    MoveStatus status = g_movements.getStatus();
    if (status == last_status) return;
    if (status == MoveStatus::kDone) {
        Serial.println("done");
        printTelemetry();
    } else if (status == MoveStatus::kTimeout) {
        Serial.println("timeout: move aborted");
        printTelemetry();
    }
    last_status = status;
}


void setup() {
    Serial.begin(Comms::kUsbBaudRate);

    printHelp();

    if(kSpinMotorsAtStart) {
        for (uint8_t i = 0; i < MotorID::kCount; i++) spinMotor(i);
    }

    g_chassis.begin(); //Motor and PID setup for each wheel
    if (!g_imu.begin()) {
        Serial.println("IMU failed to initialize");
    }

     // Wait for the first heading, so the robot starts with heading 0.
    uint32_t wait_start_ms = millis();
    while (!g_imu.isReady() && millis() - wait_start_ms < kImuWaitMs) {
        g_imu.update();
    }

    g_movements.begin(&g_chassis, &g_imu); //Connect the Movements class to the Chassis class

    Serial.println(g_imu.isReady() ? "start" : "no heading, sequence disabled");
    last_loop_us = micros();
}

void loop() {
    // Fixed period control loop. Nothing in here blocks or waits.
    uint32_t now_us = micros();
    if (now_us - last_loop_us < Control::kPeriodMs * 1000UL) return;

    float dt = (now_us - last_loop_us) / 1000000.0f;
    last_loop_us = now_us;

    readSerialCommands();     // 1. Orders typed in the monitor
    g_imu.update();           // 2. Read the heading
    g_chassis.update(dt);     // 3. Wheel speed PIDs
    g_movements.update(dt);   // 4. Advance / turn logic
    reportStatusChange();     // 5. Say "done" when a move ends

    // Telemetry only while something is moving, so the monitor stays readable.
    if (g_movements.isBusy() && millis() - last_print_ms >= kPrintPeriodMs) {
        last_print_ms = millis();
        printTelemetry();   
    }
}