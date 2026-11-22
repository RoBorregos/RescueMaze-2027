#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

class Motor {
    private:
        uint8_t in1_pin;
        uint8_t in2_pin;
        uint8_t pwm_pin;          // Sets how much power the motor gets
        uint8_t enc_c1_pin;       // Encoder channel 1 (interrupt pin)
        uint8_t enc_c2_pin;       // Encoder channel 2 (gives the direction)
        bool motor_inverted;      // True if positive power spins the wheel backwards
        bool encoder_inverted;    // True if moving forward counts negative
        float cm_per_tick;        // Cm traveled per encoder tick

        int power;                // Last power sent, -255 to 255
        volatile int32_t ticks;   // Written by the interrupt, so it is volatile
        int32_t last_ticks;       // Ticks at the previous update()
        float speed_cm_s;         // Filtered speed

        int32_t readTicks() const;

    public:
        void initialize(uint8_t in1_pin_, uint8_t in2_pin_, uint8_t pwm_pin_,
                        uint8_t enc_c1_pin_, uint8_t enc_c2_pin_,
                        bool motor_inverted_, bool encoder_inverted_, float cm_per_tick_);
        void setPower(int power_);  // Power is between -255 and 255
        void brake();
        void countTick();           // Called by the interrupt
        void update(float dt);      // dt: seconds since last call
        void resetDistance();

        int getPower() const;
        float getSpeed() const;     // Returns speed in cm/s
        float getDistance() const;  // Since last reset, in cm
};

#endif
