#ifndef MOVEMENTS_H
#define MOVEMENTS_H

#include <stdint.h>
#include "chassis.h"

enum class MoveType { kNone, kAdvance, kTurn };
enum class MoveStatus { kIdle, kRunning, kDone, kTimeout };

class Movements {
    private:
        Chassis* chassis;
        MoveType type;
        MoveStatus status;
        float target; // cm
        uint32_t start_ms; // ms

    public:
        void begin(Chassis* chassis_);
        void startAdvance(float distance_cm);
        void startTurn(float degrees);
        void update();
        void cancel();
        bool isBusy() const;
        MoveStatus getStatus() const;
};

#endif