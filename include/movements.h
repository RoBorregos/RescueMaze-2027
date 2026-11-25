#ifndef MOVEMENTS_H
#define MOVEMENTS_H

#include <stdint.h>
#include "motors.h"

enum class MoveType { kNone, kAdvance, kTurn };
enum class MoveStatus { kIdle, kRunning, kDone, kTimeout, kPaused };

class Movements {
    private:
        Chassis* chassis;
        MoveType type;
        MoveStatus status;
        float target;           // Centimeters for an advance, degrees for a turn.
        uint32_t start_ms;
        uint32_t timeout_ms;
        float distance_tolerance_cm;
        float advance_speed_cm_s;
        float requested_advance_cm = 0.0f;
        bool turn_settling = false;
        bool turn_bno_only = false;
        uint32_t turn_settle_start_ms = 0;
        int8_t turn_direction = 0;
        void updateImpl(bool useHeading, float headingErrorDegrees, const float *advanceDistanceCm = nullptr);

    public:
        void begin(Chassis* chassis_);
        void startAdvance(float distance_cm, uint32_t timeout_ms_ = Motion::kMoveTimeoutMs,
                          float distance_tolerance_cm_ = Motion::kDistanceToleranceCm,
                          float max_speed_cm_s = Motion::kCruiseSpeedCmS);
        void startTurn(float degrees, uint32_t timeout_ms_ = Motion::kMoveTimeoutMs); // Positive angles turn clockwise.
        void update();
        void updateWithHeading(float headingErrorDegrees);
        void updateAdvanceWithHeading(float headingErrorDegrees, float traveledCm);
        bool setForwardAdvanceLimit(float distanceCm);
        void pause();
        bool resume();
        void cancel();
        bool isBusy() const;
        bool isTurnSettling() const;
        MoveStatus getStatus() const;
};

#endif
