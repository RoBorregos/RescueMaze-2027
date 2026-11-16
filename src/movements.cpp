#include "movements.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

void Movements::begin(Chassis* chassis_) {
    chassis = chassis_;
    type = MoveType::kNone;
    status = MoveStatus::kIdle;
    target = 0.0f;
    start_ms = 0;
}

void Movements::startAdvance(float distance_cm) {
    if (status == MoveStatus::kRunning) return;  // Ignore if already running

    // Nothing to do if the distance is too small
    if (fabs(distance_cm) < 0.01f) {
        status = MoveStatus::kDone;
        return;
    }

    chassis->resetDistance();
    type = MoveType::kAdvance;
    target = distance_cm;
    start_ms = millis();
    status = MoveStatus::kRunning;

    float speed = (distance_cm > 0) ? Motion::kCruiseSpeedCmS : -Motion::kCruiseSpeedCmS;
    chassis->setTargetWheelSpeeds(speed, speed);
}

void Movements::startTurn(float degrees) {
    if (status == MoveStatus::kRunning) return;  // Ignore if already running

    // Nothing to do if the angle is too small
    if (fabs(degrees) < 0.5f) {
        status = MoveStatus::kDone;
        return;
    }

    chassis->resetDistance();
    type = MoveType::kTurn;
    target = degrees;
    start_ms = millis();
    status = MoveStatus::kRunning;

    // Clockwise (+): left wheels forward, right wheels backward
    float speed = Motion::kTurnSpeedCmS;
    if (degrees > 0) {
        chassis->setTargetWheelSpeeds(speed, -speed);
    } else {
        chassis->setTargetWheelSpeeds(-speed, speed);
    }
}

void Movements::update() {
    if (status != MoveStatus::kRunning) return;

    // Abort a move that takes too long (stuck robot, bad encoder...)
    if (millis() - start_ms > Motion::kMoveTimeoutMs) {
        chassis->stop();
        status = MoveStatus::kTimeout;
        type = MoveType::kNone;
        return;
    }

    float sign = (target > 0.0f) ? 1.0f : -1.0f;  // Direction of the move

    // Each move type fills these 4 values, the rest is shared
    float remaining = 0.0f;   // How much is left to do (cm of wheel travel)
    float max_speed = 0.0f;   // Speed limit for this move (cm/s)
    float left_sign = 0.0f;   // Direction of the left wheels
    float right_sign = 0.0f;  // Direction of the right wheels

    if (type == MoveType::kAdvance) {
        float distance = (chassis->getLeftDistance() + chassis->getRightDistance()) / 2.0f;
        remaining = fabs(target) - sign * distance;
        max_speed = Motion::kCruiseSpeedCmS;
        left_sign = sign;
        right_sign = sign;
    } else if (type == MoveType::kTurn) {
        // Distance each wheel must travel to turn the requested angle
        float arc = Geometry::kPi * Geometry::kTrackWidthCm * fabs(target) / 360.0f;
        // Clockwise turn: left wheels go forward (+), right wheels backward (-)
        float progress = (chassis->getLeftDistance() - chassis->getRightDistance()) / 2.0f;
        remaining = arc - sign * progress;
        max_speed = Motion::kTurnSpeedCmS;
        left_sign = sign;
        right_sign = -sign;
    } else {
        return;
    }

    if (remaining <= Motion::kDistanceToleranceCm) {
        chassis->stop();
        status = MoveStatus::kDone;
        type = MoveType::kNone;
        return;
    }

    // Slow down as the goal gets closer
    float speed = constrain(Motion::kDecelGain * remaining, Motion::kMinSpeedCmS, max_speed);
    chassis->setTargetWheelSpeeds(left_sign * speed, right_sign * speed);
}

void Movements::cancel() {
    chassis->stop();
    type = MoveType::kNone;
    status = MoveStatus::kIdle;
}

bool Movements::isBusy() const {
    return status == MoveStatus::kRunning;
}

MoveStatus Movements::getStatus() const {
    return status;
}
