#include "Encoder.h"
#include "config.h"
#include <Arduino.h>

void setup() {
  Serial.begin(Comms::kUsbBaudRate);
  robot.setupMotors();
  robot.stop();

  robot.screenBegin();
  Serial.println("=== RoboCup Rescue Maze ===");
  robot.screenPrint("READY");

  if (Pins::checkpointPin != Pins::kUnassignedPin) {
    pinMode(Pins::checkpointPin, INPUT_PULLDOWN);
  }
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontLeft]),
                  Interrupts::frontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontRight]),
                  Interrupts::frontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackLeft]),
                  Interrupts::backLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackRight]),
                  Interrupts::backRightEncoder, RISING);
}

void loop() {}
