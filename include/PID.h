#ifndef PID_H
#define PID_H

#include <stdint.h>

class PID {
private:
    float kp = 6.0f;
    float ki = 0.1f;
    float kd = 0.8f;
    float lastError = 0.0f;
    uint32_t lastUpdateMs = 0;

public:
    PID() = default;
    PID(float kp, float ki, float kd, float periodMs);
    double calculate_PID(float setpoint, float input);
    void changeConstants(float kp, float ki, float kd, float periodMs);
    void reset();
    float minimumPeriodMs = 20.0f;
};

#endif
