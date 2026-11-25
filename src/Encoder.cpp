#include "Encoder.h"

#include <Arduino.h>

#include "config.h"
#include "motor.h"

namespace {
Motor *encoderMotors;
}

void EncoderInterrupts::beginEncoders(Motor *motors) {
    encoderMotors = motors;
    attachInterrupt(digitalPinToInterrupt(Pins::kEncoderC1[MotorID::kFrontLeft]), frontLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Pins::kEncoderC1[MotorID::kFrontRight]), frontRightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Pins::kEncoderC1[MotorID::kBackRight]), backRightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Pins::kEncoderC1[MotorID::kBackLeft]), backLeftEncoder, CHANGE);
}

void EncoderInterrupts::backRightEncoder() { encoderMotors[MotorID::kBackRight].updateTicks(); }
void EncoderInterrupts::backLeftEncoder() { encoderMotors[MotorID::kBackLeft].updateTicks(); }
void EncoderInterrupts::frontRightEncoder() { encoderMotors[MotorID::kFrontRight].updateTicks(); }
void EncoderInterrupts::frontLeftEncoder() { encoderMotors[MotorID::kFrontLeft].updateTicks(); }
