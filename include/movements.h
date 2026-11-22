#ifndef MOVEMENTS_H
#define MOVEMENTS_H

#include <stdint.h>
#include "chassis.h"

enum class MoveType { kNone, kAdvance, kTurn };
enum class MoveStatus { kIdle, kRunning, kDone, kTimeout };

class Movements {
    private:
        Chassis* chassis;       // Pointer to the shared Chassis (not a copy)
        MoveType type;
        MoveStatus status;
        float target;           // cm for advance, degrees for turn
        uint32_t start_ms;      // When the move started

    public:
        void begin(Chassis* chassis_);
        void startAdvance(float distance_cm);
        void startTurn(float degrees);   // + = clockwise (right), - = counter-clockwise
        void update();                   // Call every control cycle
        void cancel();                   // Stop the current move
        bool isBusy() const;
        MoveStatus getStatus() const;
};

#endif
