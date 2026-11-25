#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

class Motor {
    private:
        uint8_t in1_pin;
        uint8_t in2_pin;
        uint8_t pwm_pin;
        uint8_t enc_c1_pin;
        uint8_t enc_c2_pin;
        bool motor_inverted;      // True if positive power spins the wheel backwards
        bool encoder_inverted;    // True if moving forward counts negative
        float cm_per_tick;

        int power;
        volatile int32_t ticks;   // Updated by the encoder interrupt.
        int32_t last_ticks;
        uint32_t last_sample_ms;
        float speed_cm_s;

        int32_t readTicks() const;

    public:
        void initialize(uint8_t in1_pin_, uint8_t in2_pin_, uint8_t pwm_pin_,
                        uint8_t enc_c1_pin_, uint8_t enc_c2_pin_,
                        bool motor_inverted_, bool encoder_inverted_, float cm_per_tick_);
        void setPower(int power_);
        void brake();
        void updateTicks();
        bool update(float dt);      // Returns true when a new speed sample is available.
        void resetDistance();
        int32_t getTicks() const;
        float getSingleEdgeTicksPerWindow() const;

        int getPower() const;
        float getSpeed() const;     // cm/s
        float getDistance() const;  // cm since the last reset
};

#endif
