#include "movements.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

void Movements::begin(Chassis* chassis_,    Imu* imu_) {
    chassis = chassis_;
    imu = imu_;
    turn_pid = PID(Motion::kTurnKp, 0.0f, Motion::kTurnKd, Motion::kTurnSpeedCmS);
    heading_pid = PID(Motion::kHeadingKp, 0.0f, Motion::kHeadingKd, Motion::kMaxHeadingCorrectionCmS);
    type = MoveType::kNone;
    status = MoveStatus::kIdle;
    target_distance_cm = 0.0f;
    target_heading_deg = imu->getHeading();
    start_ms = 0;
    settled_cycles = 0;
}

void Movements::startAdvance(float distance_cm) {
    if (status == MoveStatus::kRunning) return;  // Ignore if already running

    // Nothing to do if the distance is too small
    if (fabs(distance_cm) < 0.01f) {
        status = MoveStatus::kDone;
        return;
    }

    //target_heading_deg does not change.
    chassis->resetDistance();
    heading_pid.reset();
    type = MoveType::kAdvance;
    target_distance_cm = distance_cm;
    start_ms = millis();
    status = MoveStatus::kRunning;
}

void Movements::startTurn(float degrees) {
    if (status == MoveStatus::kRunning) return;  // Ignore if already running

    // Nothing to do if the angle is too small
    if (fabs(degrees) < 0.5f) {
        status = MoveStatus::kDone;
        return;
    }

    // The new target is the previous TARGET plus the turn. +90 turns the target is exactly 360.
    target_heading_deg += degrees;
    turn_pid.reset();
    settled_cycles = 0;
    type = MoveType::kTurn;
    start_ms = millis();
    status = MoveStatus::kRunning;
}

void Movements::update(float dt) {
    if (status != MoveStatus::kRunning) return;

    // Abort a move that takes too long (stuck robot, bad sensor...)
    if (millis() - start_ms > Motion::kMoveTimeoutMs) {
        finish(MoveStatus::kTimeout);
        return;
    }

    float heading = imu->getHeading();
    if (type == MoveType::kAdvance) {
        updateAdvance(heading, dt);
    } else if (type == MoveType::kTurn) {
        updateTurn(heading, dt);
    }
}

void Movements::updateAdvance(float heading, float dt) {
    float sign = (target_distance_cm > 0.0f) ? 1.0f : -1.0f;  // Forward or backward
    float distance = (chassis->getLeftDistance() + chassis->getRightDistance()) / 2.0f;
    float remaining = fabs(target_distance_cm) - sign * distance;

    if (remaining <= Motion::kDistanceToleranceCm) {
        finish(MoveStatus::kDone);
        return;
    }

    // Base speed: cruise, slowing down near the goal
    float speed = sign * constrain(Motion::kDecelGain * remaining, Motion::kMinSpeedCmS, Motion::kCruiseSpeedCmS);

    float correction = heading_pid.calculate_PID(target_heading_deg, heading, dt);
    chassis->setTargetWheelSpeeds(speed + correction, speed - correction);
}

void Movements::updateTurn(float heading, float dt){
    float error = target_heading_deg - heading;

    if (fabs(error) <= Motion::kTurnToleranceDeg) {
        settled_cycles++;
        chassis->setTargetWheelSpeeds(0.0f, 0.0f);
        if (settled_cycles >= Motion::kTurnSettleCycles) {
            finish(MoveStatus::kDone);
        }
        return;
    }
    settled_cycles = 0;

    float speed = turn_pid.calculate_PID(target_heading_deg, heading, dt);
    if (fabs(speed) < Motion::kMinTurnSpeedCmS) {
        speed = (error > 0.0f) ? Motion::kMinTurnSpeedCmS : -Motion::kMinTurnSpeedCmS;
    }

    // Clockwise (+): left wheels forward, right wheels backward
    chassis->setTargetWheelSpeeds(speed, -speed);
}

void Movements::finish(MoveStatus final_status) {
    chassis->stop();
    type = MoveType::kNone;
    status = final_status;
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

float Movements::getTargetHeading() const {
    return target_heading_deg;
}
