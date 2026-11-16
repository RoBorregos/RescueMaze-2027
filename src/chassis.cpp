#include "chassis.h"
#include <Arduino.h>

static void IRAM_ATTR encoderIsr(void* arg) {
    static_cast<Motor*>(arg)->countTick();
}

void Chassis::begin() {
    
    stopped = false;
    for (uint8_t i = 0; i < MotorID::kCount; i++) {

        motors[i].initialize(
            Pins::kMotorIn1[i], 
            Pins::kMotorIn2[i], 
            Pins::kMotorPwm[i], 
            Pins::kMotorInverted[i], 
            Geometry::kCmPerTick
        );

        pids[i] = PID(
            Control::kWheelKp, 
            Control::kWheelKi, 
            Control::kWheelKd, 
            Control::kPwmMax
        );

        target_speeds[i] = 0.0f;

        pinMode(Pins::kEncoderA[i], INPUT); // GPIO 34, 35, 36 and 39 have no internal pullup. Set encoder pin as input
        attachInterruptArg(Pins::kEncoderA[i], encoderIsr, &motors[i], RISING);
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
