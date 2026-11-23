#ifndef MOVEMENTS_H
#define MOVEMENTS_H

#include <stdint.h>
#include "chassis.h"
#include "imu.h"
#include "pid.h"

enum class MoveType { kNone, kAdvance, kTurn };
enum class MoveStatus { kIdle, kRunning, kDone, kTimeout };

class Movements {
    private:
        Chassis* chassis;       // Pointer to the shared Chassis (not a copy)
        Imu* imu;               // Pointer to the shared Imu
        PID turn_pid;           // Heading error -> turning wheel speed
        PID heading_pid;        // Heading error -> Small correction while advancing
        MoveType type;
        MoveStatus status;
        float target_distance_cm;    // cm for advance
        float target_heading_deg;  // Heading the robot must turn or advance, degrees
        uint32_t start_ms;      // When the move started
        uint8_t settled_cycles;    // Cycles the turn has been inside the tolerance

        void finish(MoveStatus final_status);
        void updateAdvance(float heading, float dt);
        void updateTurn(float heading, float dt);

    public:
        void begin(Chassis* chassis_, Imu* imu);
        void startAdvance(float distance_cm); 
        void startTurn(float degrees);   // + = clockwise (right), - = counter-clockwise
        void update(float dt);                   // Call every control cycle
        void cancel();                   // Stop the current move
        bool isBusy() const;
        MoveStatus getStatus() const;
        float getTargetHeading() const;
};

#endif
