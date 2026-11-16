#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>

class Motor {
    private:
        uint8_t in1_pin;
        uint8_t in2_pin; 
        uint8_t pwm_pin; //Alimenation
        bool inverted; //True if the motor is inverted
        float cm_per_tick; //Cm traveled per encoder tick

        int power; // Last power sent, -255 to 255
        volatile int32_t ticks; //written by the interrput , so it is volatile
        int32_t last_ticks; //ticks at the previous update()
        float speed_cm_s; // Actual speed/cm/s 

    public:
        void initialize(uint8_t in1_pin_, uint8_t in2_pin_, uint8_t pwm_pin_, bool inverted_, float cm_per_tick_);
        void setPower(int power_); //Power is between -255 and 255
        void brake();
        void countTick(); //Called by the interrupt
        void update(float dt); //dt: seconds since last call
        void resetDistance();

        int getPower() const;
        float getSpeed() const; //Returns speed in cm/s
        float getDistance() const; // Since last reset, in cm
};

#endif