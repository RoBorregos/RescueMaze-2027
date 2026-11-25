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
    timeout_ms = Motion::kMoveTimeoutMs;
    distance_tolerance_cm = Motion::kDistanceToleranceCm;
    advance_speed_cm_s = Motion::kCruiseSpeedCmS;
}

void Movements::startAdvance(float distance_cm, uint32_t timeout_ms_,
                             float distance_tolerance_cm_, float max_speed_cm_s) {
    if (status == MoveStatus::kRunning) return;

    if (fabs(distance_cm) < 0.01f) {
        status = MoveStatus::kDone;
        return;
    }

    chassis->resetDistance();
    type = MoveType::kAdvance;
    target = distance_cm;
    requested_advance_cm = distance_cm;
    start_ms = millis();
    timeout_ms = timeout_ms_;
    distance_tolerance_cm = distance_tolerance_cm_;
    advance_speed_cm_s = max(Motion::kMinSpeedCmS, max_speed_cm_s);
    status = MoveStatus::kRunning;

    const float speed = copysignf(advance_speed_cm_s, distance_cm);
    chassis->setTargetWheelSpeeds(speed, speed);
}

void Movements::startTurn(float degrees, uint32_t timeout_ms_) {
    if (status == MoveStatus::kRunning) return;

    if (fabs(degrees) < 0.5f) {
        status = MoveStatus::kDone;
        return;
    }

    chassis->resetDistance();
    type = MoveType::kTurn;
    target = degrees;
    start_ms = millis();
    timeout_ms = timeout_ms_;
    distance_tolerance_cm = Motion::kDistanceToleranceCm;
    status = MoveStatus::kRunning;

    turn_settling = false;
    turn_bno_only = false;
    turn_direction = degrees > 0.0f ? 1 : -1;
    chassis->setTurnWheelSpeed(copysignf(Motion::kTurnSpeedCmS, degrees));
}

void Movements::update() {
    updateImpl(false, 0.0f);
}

void Movements::updateWithHeading(float headingErrorDegrees) {
    updateImpl(true, headingErrorDegrees);
}

void Movements::updateAdvanceWithHeading(float headingErrorDegrees, float traveledCm) {
    updateImpl(true, headingErrorDegrees, &traveledCm);
}

bool Movements::setForwardAdvanceLimit(float distanceCm) {
    if (status != MoveStatus::kRunning || type != MoveType::kAdvance || requested_advance_cm < 0.0f) return false;
    const float limited = constrain(distanceCm, 0.0f, requested_advance_cm);
    if (limited == target) return false;
    target = limited;
    return true;
}

void Movements::updateImpl(bool useHeading, float headingErrorDegrees, const float *advanceDistanceCm) {
    if (status != MoveStatus::kRunning) return;

    const uint32_t now = millis();
    if (now - start_ms >= timeout_ms) {
        chassis->stop();
        status = MoveStatus::kTimeout;
        return;
    }

    if (useHeading && type == MoveType::kTurn) {
        constexpr uint32_t kTurnSettleMs = 150;
        constexpr float kTurnMinSpeedCmS = 2.5f;
        constexpr float kTurnSpeedPerDegree = 0.24f;
        const float error = fabsf(headingErrorDegrees);
        const int8_t direction = headingErrorDegrees > 0.0f ? 1 : -1;

        if (turn_settling) {
            if (now - turn_settle_start_ms < kTurnSettleMs) return;
            if (error <= Motion::kTurnToleranceDegrees) {
                status = MoveStatus::kDone;
                type = MoveType::kNone;
                return;
            }
            turn_settling = false;
            turn_direction = 0;
        }

        if (error <= Motion::kTurnToleranceDegrees || (turn_direction != 0 && direction != turn_direction)) {
            chassis->stop();
            turn_settling = true;
            turn_settle_start_ms = now;
            return;
        }

        turn_direction = direction;
        const float speed = constrain(kTurnSpeedPerDegree * error, kTurnMinSpeedCmS, Motion::kTurnSpeedCmS);
        turn_bno_only = turn_bno_only || !chassis->hasEncoderOnEachSide();
        chassis->setTurnWheelSpeed(direction * speed, turn_bno_only);
        return;
    }

    const float sign = (target >= 0.0f) ? 1.0f : -1.0f;

    float remaining = 0.0f;   // cm of wheel travel
    float max_speed = 0.0f;   // cm/s

    if (type == MoveType::kAdvance) {
        const float distance = advanceDistanceCm != nullptr ? *advanceDistanceCm :
            (chassis->getLeftDistance() + chassis->getRightDistance()) / 2.0f;
        remaining = fabs(target) - sign * distance;
        max_speed = advance_speed_cm_s;
    } else if (type == MoveType::kTurn) {
        // Each side traces an arc with a radius of half the track width.
        float arc = Geometry::kPi * Geometry::kTrackWidthCm * fabs(target) / 360.0f;
        float progress = chassis->getTurnProgress();
        remaining = arc - sign * progress;
        max_speed = Motion::kTurnSpeedCmS;
    } else {
        return;
    }

    if (remaining <= distance_tolerance_cm) {
        chassis->stop();
        status = MoveStatus::kDone;
        type = MoveType::kNone;
        return;
    }

    float speed = constrain(Motion::kDecelGain * remaining, Motion::kMinSpeedCmS, max_speed);
    if (type == MoveType::kTurn) {
        chassis->setTurnWheelSpeed(sign * speed);
    } else if (useHeading) {
        constexpr float kHeadingGain = 0.50f;          // cm/s of correction per degree
        constexpr float kSlowdownPerDegree = 0.045f;
        constexpr float kMaxCorrectionFraction = 0.60f;
        const float headingError = fabsf(headingErrorDegrees);
        const float guidedSpeed = max(Motion::kMinSpeedCmS, speed / (1.0f + kSlowdownPerDegree * headingError));
        const float correctionLimit = kMaxCorrectionFraction * guidedSpeed;
        const float correction = constrain(kHeadingGain * headingErrorDegrees,
                                           -correctionLimit, correctionLimit);
        const float scale = min(1.0f, max_speed / (guidedSpeed + fabsf(correction)));
        chassis->setTargetWheelSpeeds((guidedSpeed * sign + correction) * scale,
                                      (guidedSpeed * sign - correction) * scale);
    } else {
        chassis->setTargetWheelSpeeds(sign * speed, sign * speed);
    }
}

void Movements::pause() {
    if (status != MoveStatus::kRunning) return;
    chassis->stop();
    status = MoveStatus::kPaused;
}

bool Movements::resume() {
    if (status != MoveStatus::kTimeout && status != MoveStatus::kPaused) return false;
    start_ms = millis();
    status = MoveStatus::kRunning;
    return true;
}

void Movements::cancel() {
    chassis->stop();
    type = MoveType::kNone;
    status = MoveStatus::kIdle;
}

bool Movements::isBusy() const {
    return status == MoveStatus::kRunning;
}

bool Movements::isTurnSettling() const {
    return status == MoveStatus::kRunning && type == MoveType::kTurn && turn_settling;
}

MoveStatus Movements::getStatus() const {
    return status;
}
