    #include "movements.h"
    #include "config.h"
    #include <Arduino.h>
    #include <math.h>

    void Movements::begin(Chassis* chassis_, Imu* imu_) {
        chassis = chassis_;
        imu = imu_;
        turn_pid = PID(Motion::kTurnKp, 0.0f, Motion::kTurnKd, Motion::kTurnSpeedCmS);
        heading_pid = PID(Motion::kHeadingKp, 0.0f, Motion::kHeadingKd,
                        Motion::kMaxHeadingCorrectionCmS);
        type = MoveType::kNone;
        status = MoveStatus::kIdle;
        target_distance_cm = 0.0f;
        target_heading_deg = imu->getHeading();  // Keep the heading the robot starts with
        turn_size_deg = 0.0f;
        start_ms = 0;
        settled_cycles = 0;
    }

    void Movements::startAdvance(float distance_cm) {
        if (isBusy()) return;  // Ignore if a move is running or paused

        // Nothing to do if the distance is too small
        if (fabs(distance_cm) < 0.01f) {
            status = MoveStatus::kDone;
            return;
        }

        // target_heading_deg does not change: the robot keeps the heading it has.
        chassis->resetDistance();
        heading_pid.reset();
        type = MoveType::kAdvance;
        target_distance_cm = distance_cm;
        start_ms = millis();
        status = MoveStatus::kRunning;
    }

    void Movements::startTurn(float degrees) {
        if (isBusy()) return;  // Ignore if a move is running or paused

        // Nothing to do if the angle is too small
        if (fabs(degrees) < 0.5f) {
            status = MoveStatus::kDone;
            return;
        }

        // The new target is the previous TARGET plus the turn, not the current
        // heading plus the turn. That way small errors never add up: after four
        // +90 turns the target is exactly 360.
        target_heading_deg += degrees;
        turn_size_deg = fabs(degrees);
        turn_pid.reset();
        settled_cycles = 0;
        type = MoveType::kTurn;
        start_ms = millis();
        status = MoveStatus::kRunning;
    }

    // Pausing only brakes the motors. The distance counted since the move started
    // and the target heading are kept, so resume() just lets update() continue
    // from where it was.
    bool Movements::pause() {
        if (status != MoveStatus::kRunning) return false;
        chassis->stop();
        status = MoveStatus::kPaused;
        return true;
    }

    bool Movements::resume() {
        if (status != MoveStatus::kPaused) return false;
        turn_pid.reset();
        heading_pid.reset();
        settled_cycles = 0;
        start_ms = millis();  // The paused time does not count for the timeout
        status = MoveStatus::kRunning;
        return true;
    }

    void Movements::cancel() {
        chassis->stop();
        type = MoveType::kNone;
        status = MoveStatus::kIdle;
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
        float speed = sign * constrain(Motion::kDecelGain * remaining,
                                    Motion::kMinSpeedCmS, Motion::kCruiseSpeedCmS);

        // Heading correction: positive means "rotate clockwise", so the left
        // wheels go a bit faster and the right wheels a bit slower.
        float correction = heading_pid.calculate_PID(target_heading_deg, heading, dt);
        chassis->setTargetWheelSpeeds(speed + correction, speed - correction);
    }

    void Movements::updateTurn(float heading, float dt) {
        float error = target_heading_deg - heading;

        // Close enough: hold still and wait a few cycles to make sure it stays there.
        if (fabs(error) <= Motion::kTurnToleranceDeg) {
            settled_cycles++;
            chassis->setTargetWheelSpeeds(0.0f, 0.0f);
            if (settled_cycles >= Motion::kTurnSettleCycles) {
                finish(MoveStatus::kDone);
            }
            return;
        }
        settled_cycles = 0;

        // Wheel speed proportional to the error: fast when far, slow when close.
        float speed = turn_pid.calculate_PID(target_heading_deg, heading, dt);

        // Too slow and the wheels do not move at all, so keep a minimum.
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

    bool Movements::isBusy() const {
        return status == MoveStatus::kRunning || status == MoveStatus::kPaused;
    }

    bool Movements::isPaused() const {
        return status == MoveStatus::kPaused;
    }

    MoveStatus Movements::getStatus() const {
        return status;
    }

    float Movements::getProgress() const {
        if (status == MoveStatus::kDone) return 1.0f;
        if (!isBusy()) return 0.0f;

        float progress = 0.0f;
        if (type == MoveType::kAdvance) {
            float distance = (chassis->getLeftDistance() + chassis->getRightDistance()) / 2.0f;
            progress = fabs(distance) / fabs(target_distance_cm);
        } else if (type == MoveType::kTurn) {
            float error = fabs(target_heading_deg - imu->getHeading());
            progress = 1.0f - error / turn_size_deg;
        }
        return constrain(progress, 0.0f, 1.0f);
    }

    float Movements::getTargetHeading() const {
        return target_heading_deg;
    }