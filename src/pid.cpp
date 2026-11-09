#include "pid.h"

PID::PID() {
  kp = 0;
  ki = 0;
  kd = 0;
  output_limit = 255;
  reset();
}

PID::PID(float kp_, float ki_, float kd_, float output_limit_) {
    kp = kp_;
    ki = ki_;  
    kd = kd_;
    output_limit = output_limit_;
    reset();
}

void PID::changeConstants(float kp_, float ki_, float kd_) {
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void PID::reset(){
    integral = 0;
    last_measurement = 0;
    first_call = true;
}

// Returns the PID output after checking if it passes the filter
float PID::limit(float value) {
    if (value > output_limit){
        return output_limit;
    } else if (value < -output_limit){
        return -output_limit;
    } else {
        return value;
    }
}

float PID::calculate_PID(float target, float measurement, float dt){
    if (dt <= 0) return 0; //Safe | Do not divide with 0

    float error = target - measurement;

    float proportional = kp * error;

    integral = integral + ki * error * dt;
    integral = limit(integral); //Anti-windup

    float derivative = 0;
    if (!first_call) {
        derivative = -kd * (measurement - last_measurement) / dt;
    }
    last_measurement = measurement;
    first_call = false;

    float output = proportional + integral + derivative;
    return limit(output);
}