#include "PID.h"

#include <Arduino.h>

PID::PID(float kp_, float ki_, float kd_, float periodMs) {
    changeConstants(kp_, ki_, kd_, periodMs);
}

void PID::changeConstants(float kp_, float ki_, float kd_, float periodMs) {
    kp = kp_;
    ki = ki_;
    kd = kd_;
    minimumPeriodMs = periodMs < 1.0f ? 1.0f : periodMs;
    reset();
}

void PID::reset() {
    lastError = 0.0f;
    lastUpdateMs = millis();
}

double PID::calculate_PID(float setpoint, float input) {
    const uint32_t now = millis();
    const uint32_t elapsed = now - lastUpdateMs;
    if (elapsed == 0 || elapsed < minimumPeriodMs) return 0.0;

    // The 2026 controller uses the sum of the two latest errors for I.
    // Its result is a correction to the previous PWM, with time in ms.
    const float error = setpoint - input;
    const float proportional = kp * error;
    const float integral = ki * (error + lastError);
    const float derivative = kd * (error - lastError) / elapsed;
    lastError = error;
    lastUpdateMs = now;
    return proportional + integral + derivative;
}
