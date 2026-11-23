#ifndef MOVEMENTS_H
#define MOVEMENTS_H

#include <stdint.h>
#include "chassis.h"
#include "imu.h"
#include "pid.h"

enum class MoveType { kNone, kAdvance, kTurn };
enum class MoveStatus { kIdle, kRunning, kPaused, kDone, kTimeout };

class Movements {
    private:
        Chassis* chassis;          // Pointer to the shared Chassis (not a copy)
        Imu* imu;                  // Pointer to the shared Imu
        PID turn_pid;              // Heading error -> turning wheel speed
        PID heading_pid;           // Heading error -> small correction while advancing
        MoveType type;
        MoveStatus status;
        float target_distance_cm;  // Only for advance
        float target_heading_deg;  // Heading the robot must reach (turn) or keep (advance)
        float turn_size_deg;       // Size of the current turn, only to report progress
        uint32_t start_ms;         // When the move started
        uint8_t settled_cycles;    // Cycles the turn has been inside the tolerance

        void finish(MoveStatus final_status);
        void updateAdvance(float heading, float dt);
        void updateTurn(float heading, float dt);

    public:
        void begin(Chassis* chassis_, Imu* imu_);
        void startAdvance(float distance_cm);  // + forward, - backward, keeps the heading
        void startTurn(float degrees);         // + = clockwise (right), - = counter-clockwise
        bool pause();                          // Brake and wait. False if nothing is running
        bool resume();                         // Continue the paused move. False if nothing is paused
        void cancel();                         // Stop and forget the current move
        void update(float dt);                 // Call every control cycle

        bool isBusy() const;                   // True while a move is running or paused
        bool isPaused() const;
        MoveStatus getStatus() const;
        float getProgress() const;             // 0.0 at the start of the move, 1.0 when done
        float getTargetHeading() const;
};

#endif
