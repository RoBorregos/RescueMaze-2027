#include "Encoder.h"
#include "motors.h"
motors robot;

void Interrupts::backRightEncoder() {
  robot.motor[MotorID::kBackRight].updateTics();
}
void Interrupts::backLeftEncoder() {
  robot.motor[MotorID::kBackLeft].updateTics();
}
void Interrupts::frontRightEncoder() {
  robot.motor[MotorID::kFrontRight].updateTics();
}
void Interrupts::frontLeftEncoder() {
  robot.motor[MotorID::kFrontLeft].updateTics();
}
void Interrupts::lackOfProgress() {
  if ((millis() - robot.buttonTime) > Comms::kButtonDebounceMs) {
    robot.buttonPressed = !robot.buttonPressed;
    robot.buttonTime = millis();
  }
}
