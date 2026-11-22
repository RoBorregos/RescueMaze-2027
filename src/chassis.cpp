#include "chassis.h"
#include "config.h"
#include <Arduino.h>

// The interrupt can not call a class method directly, so each motor gets a
// small function that forwards the encoder edge to its countTick().
static Motor* isr_motors[MotorID::kCount];
static void encoderIsr0() { isr_motors[0]->countTick(); }
static void encoderIsr1() { isr_motors[1]->countTick(); }
static void encoderIsr2() { isr_motors[2]->countTick(); }
static void encoderIsr3() { isr_motors[3]->countTick(); }
static void (*const kEncoderIsr[MotorID::kCount])() = {
    encoderIsr0, encoderIsr1, encoderIsr2, encoderIsr3
};

void Chassis::begin() {

    // PWM resolution is global
    analogWriteResolution(Control::kPwmResolutionBits);

    stopped = false;
    for (uint8_t i = 0; i < MotorID::kCount; i++) {

        motors[i].initialize(
            Pins::kMotorIn1[i],
            Pins::kMotorIn2[i],
            Pins::kMotorPwm[i],
            Pins::kEncoderC1[i],
            Pins::kEncoderC2[i],
            Pins::kMotorInverted[i],
            Pins::kEncoderInverted[i],
            Geometry::kCmPerTick
        );

        pids[i] = PID(
            Control::kWheelKp,
            Control::kWheelKi,
            Control::kWheelKd,
            Control::kPwmMax
        );

        target_speeds[i] = 0.0f;

        isr_motors[i] = &motors[i];
        attachInterrupt(digitalPinToInterrupt(Pins::kEncoderC1[i]), kEncoderIsr[i], CHANGE);
    }
}

void Chassis::update(float dt) {
    for (uint8_t i = 0; i < MotorID::kCount; i++) {
        motors[i].update(dt);
        if (stopped) continue;
        float pid_output = pids[i].calculate_PID(target_speeds[i], motors[i].getSpeed(), dt);
        motors[i].setPower(static_cast<int>(pid_output));
    }
}

void Chassis::setTargetWheelSpeeds(float left_cm_s, float right_cm_s) {
    stopped = false;
    target_speeds[MotorID::kFrontLeft] = left_cm_s;
    target_speeds[MotorID::kBackLeft] = left_cm_s;
    target_speeds[MotorID::kFrontRight] = right_cm_s;
    target_speeds[MotorID::kBackRight] = right_cm_s;
}

void Chassis::stop() {
    stopped = true;
    for (uint8_t i = 0; i < MotorID::kCount; i++) {
        target_speeds[i] = 0.0f;
        pids[i].reset();
        motors[i].brake();
    }
}

void Chassis::resetDistance() {
    for (uint8_t i = 0; i < MotorID::kCount; i++) {
        motors[i].resetDistance();
    }
}

float Chassis::getLeftDistance() const {
    return (motors[MotorID::kFrontLeft].getDistance() + motors[MotorID::kBackLeft].getDistance()) / 2.0f;
}

float Chassis::getRightDistance() const {
    return (motors[MotorID::kFrontRight].getDistance() + motors[MotorID::kBackRight].getDistance()) / 2.0f;
}

float Chassis::getLeftSpeed() const {
    return (motors[MotorID::kFrontLeft].getSpeed() + motors[MotorID::kBackLeft].getSpeed()) / 2.0f;
}

float Chassis::getRightSpeed() const {
    return (motors[MotorID::kFrontRight].getSpeed() + motors[MotorID::kBackRight].getSpeed()) / 2.0f;
}
