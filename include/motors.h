#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"
#include "motor.h"
#include "PID.h"

struct WheelTelemetry {
    float distance_cm;
    float speed_cm_s;
    int pwm;
};

class Chassis {
    private:
        Motor motors[MotorID::kCount];
        PID pids[MotorID::kCount];
        float target_speeds[MotorID::kCount]; // cm/s
        float pwm_reference[MotorID::kCount];
        float encoderDistanceOffset[MotorID::kCount];
        int8_t wheel_direction[MotorID::kCount];
        uint8_t ignoredEncoders = 0;
        bool stopped;
        bool turning = false;
        bool turnWithoutEncoders = false;
        float averageSide(float front, float back, uint8_t frontId, uint8_t backId) const;
        float wheelDistance(uint8_t wheelId) const;

    public:
        void begin(float kp = Control::kWheelKp, float ki = Control::kWheelKi,
                   float kd = Control::kWheelKd);
        void update(float dt); // Call every control cycle with the elapsed time in seconds.
        void setTargetWheelSpeeds(float left_cm_s, float right_cm_s);
        void setTurnWheelSpeed(float clockwise_cm_s, bool withoutEncoders = false);
        void runMotorTest(uint8_t wheel_id, int pwm); // Drives one motor with closed-loop control paused.
        void stop();
        void resetDistance();
        void ignoreEncoder(uint8_t wheelId);
        bool isEncoderIgnored(uint8_t wheelId) const;
        uint8_t ignoredEncoderMask() const { return ignoredEncoders; }
        uint8_t activeEncoderCount() const;
        bool hasEncoderOnEachSide(uint8_t additionallyIgnored = 0) const;
        bool restoreEncoder(uint8_t wheelId);

        float getLeftDistance() const; // Average of the available left encoders, cm
        float getRightDistance() const;
        float getTurnProgress() const; // Average travel in the configured turn directions
        float getTurnDegrees() const;
        float getLeftSpeed() const; // Average of the available left encoders, cm/s
        float getRightSpeed() const;
        WheelTelemetry getWheelTelemetry(uint8_t wheel_id) const;
};

#endif
