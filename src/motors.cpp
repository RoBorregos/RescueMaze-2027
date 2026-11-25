#include "motors.h"
#include "Encoder.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

namespace {
uint8_t pairedWheel(uint8_t wheel) {
    if (wheel == MotorID::kFrontLeft) return MotorID::kBackLeft;
    if (wheel == MotorID::kBackLeft) return MotorID::kFrontLeft;
    if (wheel == MotorID::kFrontRight) return MotorID::kBackRight;
    return MotorID::kFrontRight;
}
}

void Chassis::begin(float kp, float ki, float kd) {

    // PWM resolution is global
    analogWriteResolution(Control::kPwmResolutionBits);

    stopped = false;
    turning = false;
    turnWithoutEncoders = false;
    ignoredEncoders = 0;
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

        pids[i].changeConstants(kp, ki, kd, Control::kWheelPidPeriodMs);

        target_speeds[i] = 0.0f;
        pwm_reference[i] = 0.0f;
        encoderDistanceOffset[i] = 0.0f;
        wheel_direction[i] = 0;
    }
    EncoderInterrupts::beginEncoders(motors);
}

void Chassis::update(float dt) {
    constexpr float ticksPerCmS = (Control::kEncoderSpeedWindowMs / 1000.0f) /
                                  (2.0f * Geometry::kCmPerTick);
    for (uint8_t i = 0; i < MotorID::kCount; i++) {
        const bool newSpeed = motors[i].update(dt);
        if (stopped || (!turnWithoutEncoders && isEncoderIgnored(i))) continue;
        const float target = target_speeds[i];
        const int8_t direction = target > 0.0f ? 1 : (target < 0.0f ? -1 : 0);
        if (direction != wheel_direction[i]) {
            pwm_reference[i] = 0.0f;
            pids[i].reset();
            wheel_direction[i] = direction;
            if (turning) motors[i].brake();
        }
        if (direction == 0) {
            motors[i].brake();
            continue;
        }
        if (turnWithoutEncoders) {
            const float fraction = constrain(fabsf(target) / Motion::kTurnSpeedCmS, 0.0f, 1.0f);
            const float desired = Motion::kBnoTurnMinPwm + fraction * (Motion::kBnoTurnMaxPwm - Motion::kBnoTurnMinPwm);
            pwm_reference[i] = constrain(pwm_reference[i], 0.0f, Motion::kBnoTurnMaxPwm);
            pwm_reference[i] += constrain(desired - pwm_reference[i], -Control::kTurnPwmStepMax, Control::kTurnPwmStepMax);
            motors[i].setPower(static_cast<int>(pwm_reference[i]) * direction);
            continue;
        }
        // During a turn, wait for the next encoder sample before changing PWM again.
        if (turning && !newSpeed) continue;
        float correction = pids[i].calculateCorrection(fabsf(target) * ticksPerCmS,
                                                       motors[i].getSingleEdgeTicksPerWindow() * direction);
        if (turning) correction = constrain(correction, -Control::kTurnPwmStepMax, Control::kTurnPwmStepMax);
        pwm_reference[i] = constrain(pwm_reference[i] + correction, 0.0f, Control::kPwmMax);
        motors[i].setPower(static_cast<int>(pwm_reference[i]) * direction);
    }
    if (!stopped && !turnWithoutEncoders) {
        for (uint8_t i = 0; i < MotorID::kCount; ++i) {
            if (!isEncoderIgnored(i)) continue;
            const uint8_t partner = pairedWheel(i);
            const int power = isEncoderIgnored(partner) ? 0 : motors[partner].getPower();
            if (power == 0) motors[i].brake();
            else motors[i].setPower(power);
        }
    }
}

void Chassis::setTargetWheelSpeeds(float left_cm_s, float right_cm_s) {
    stopped = false;
    turning = false;
    turnWithoutEncoders = false;
    target_speeds[MotorID::kFrontLeft] = left_cm_s;
    target_speeds[MotorID::kBackLeft] = left_cm_s;
    target_speeds[MotorID::kFrontRight] = right_cm_s;
    target_speeds[MotorID::kBackRight] = right_cm_s;
}

void Chassis::setTurnWheelSpeed(float clockwise_cm_s, bool withoutEncoders) {
    setTargetWheelSpeeds(clockwise_cm_s, -clockwise_cm_s);
    turning = true;
    turnWithoutEncoders = withoutEncoders;
}

void Chassis::runMotorTest(uint8_t wheel_id, int pwm) {
    stop();
    if (wheel_id < MotorID::kCount) motors[wheel_id].setPower(pwm);
}

void Chassis::stop() {
    stopped = true;
    turning = false;
    turnWithoutEncoders = false;
    for (uint8_t i = 0; i < MotorID::kCount; i++) {
        target_speeds[i] = 0.0f;
        pwm_reference[i] = 0.0f;
        wheel_direction[i] = 0;
        pids[i].reset();
        motors[i].brake();
    }
}

void Chassis::resetDistance() {
    for (uint8_t i = 0; i < MotorID::kCount; i++) {
        motors[i].resetDistance();
        encoderDistanceOffset[i] = 0.0f;
    }
}

void Chassis::ignoreEncoder(uint8_t wheelId) {
    if (wheelId < MotorID::kCount) ignoredEncoders |= 1u << wheelId;
}

bool Chassis::isEncoderIgnored(uint8_t wheelId) const {
    return wheelId < MotorID::kCount && (ignoredEncoders & (1u << wheelId));
}

uint8_t Chassis::activeEncoderCount() const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < MotorID::kCount; ++i) {
        if (!isEncoderIgnored(i)) ++count;
    }
    return count;
}

bool Chassis::hasEncoderOnEachSide(uint8_t additionallyIgnored) const {
    constexpr uint8_t left = (1u << MotorID::kFrontLeft) | (1u << MotorID::kBackLeft);
    constexpr uint8_t right = (1u << MotorID::kFrontRight) | (1u << MotorID::kBackRight);
    const uint8_t unavailable = ignoredEncoders | additionallyIgnored;
    return (unavailable & left) != left && (unavailable & right) != right;
}

bool Chassis::restoreEncoder(uint8_t wheelId) {
    if (!isEncoderIgnored(wheelId)) return false;
    const uint8_t partner = pairedWheel(wheelId);
    // Preserve the distance already estimated from the healthy wheel.
    if (!isEncoderIgnored(partner)) encoderDistanceOffset[wheelId] = wheelDistance(partner) - motors[wheelId].getDistance();
    const int power = motors[wheelId].getPower();
    pwm_reference[wheelId] = abs(power);
    wheel_direction[wheelId] = power > 0 ? 1 : (power < 0 ? -1 : 0);
    pids[wheelId].reset();
    ignoredEncoders &= ~(1u << wheelId);
    return true;
}

float Chassis::wheelDistance(uint8_t wheelId) const {
    return motors[wheelId].getDistance() + encoderDistanceOffset[wheelId];
}

float Chassis::averageSide(float front, float back, uint8_t frontId, uint8_t backId) const {
    if (isEncoderIgnored(frontId) && isEncoderIgnored(backId)) return 0.0f;
    if (isEncoderIgnored(frontId)) return back;
    if (isEncoderIgnored(backId)) return front;
    return (front + back) / 2.0f;
}

float Chassis::getLeftDistance() const {
    return averageSide(wheelDistance(MotorID::kFrontLeft), wheelDistance(MotorID::kBackLeft),
                       MotorID::kFrontLeft, MotorID::kBackLeft);
}

float Chassis::getRightDistance() const {
    return averageSide(wheelDistance(MotorID::kFrontRight), wheelDistance(MotorID::kBackRight),
                       MotorID::kFrontRight, MotorID::kBackRight);
}

float Chassis::getTurnProgress() const {
    return (getLeftDistance() - getRightDistance()) / 2.0f;
}

float Chassis::getTurnDegrees() const {
    return getTurnProgress() * 360.0f / (Geometry::kPi * Geometry::kTrackWidthCm);
}

float Chassis::getLeftSpeed() const {
    return averageSide(motors[MotorID::kFrontLeft].getSpeed(), motors[MotorID::kBackLeft].getSpeed(),
                       MotorID::kFrontLeft, MotorID::kBackLeft);
}

float Chassis::getRightSpeed() const {
    return averageSide(motors[MotorID::kFrontRight].getSpeed(), motors[MotorID::kBackRight].getSpeed(),
                       MotorID::kFrontRight, MotorID::kBackRight);
}

WheelTelemetry Chassis::getWheelTelemetry(uint8_t wheel_id) const {
    const Motor& motor = motors[wheel_id];
    return {motor.getDistance(), motor.getSpeed(), motor.getPower()};
}
