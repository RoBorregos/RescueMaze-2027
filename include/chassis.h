#ifndef CHASSIS_H
#define CHASSIS_H

#include "config.h"
#include "motor.h"
#include "pid.h"

class Chassis {
    private:
        Motor motors[MotorID::kCount];
        PID pids[MotorID::kCount];
        float target_speeds[MotorID::kCount]; //Target speeds of each wheel in cm/s
        bool stopped; //True if the chassis is stopped, false otherwise

    public:
        void begin();
        void update(float dt); // Runs the 4 wheel PIDs, call every 10 ms
        void setTargetWheelSpeeds(float left_cm_s, float right_cm_s); //Set target speeds for left and right wheels in cm/s
        void stop(); // Targets to 0, PIDs reset, motors brake
        void resetDistance();

        float getLeftDistance() const; // Average of the 2 left wheels, cm
        float getRightDistance() const;
        float getLeftSpeed() const; // Average of the 2 left wheels, cm/s
        float getRightSpeed() const;
};

#endif