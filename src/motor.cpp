#include "motor.h"
#include <Arduino.h>

void Motor::initialize(uint8_t in1_pin_, uint8_t in2_pin_, uint8_t pwm_pin_, bool inverted_, float cm_per_tick_) {
    in1_pin = in1_pin_;
    in2_pin = in2_pin_;
    pwm_pin = pwm_pin_;
    inverted = inverted_;
    cm_per_tick = cm_per_tick_;

    power = 0;
    ticks = 0;
    last_ticks = 0;
    speed_cm_s = 0.0f;

    // Set pin modes
    pinMode(in1_pin, OUTPUT);
    pinMode(in2_pin, OUTPUT);
    ledcAttach(pwm_pin, 20000, 8); // Attach PWM to the pin with 20kHz frequency and 8-bit resolution -> Stop the buzzing
    brake(); // Ensure the motor is stopped initially
}

void Motor::setPower(int power_) {
    // Clip power to [-255, 255]
    power = constrain(power_, -255, 255);
    int actual_power = inverted ? -power : power;

    if (actual_power > 0) {
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
    } else if (actual_power < 0) {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
    } else {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);
    }

    ledcWrite(pwm_pin, abs(actual_power));
}

void IRAM_ATTR Motor::countTick() {
    if (power > 0) {
        ticks++;
    } else if (power < 0) {
        ticks--;
    }
}

void Motor::update(float dt) {
    if (dt <= 0.0f) return;

    noInterrupts();
    int32_t current_ticks = ticks;
    interrupts();

    int32_t delta_ticks = current_ticks - last_ticks;
    last_ticks = current_ticks;
    float raw_speed = (delta_ticks * cm_per_tick) / dt;

    constexpr float kAlpha = 0.3f;
    speed_cm_s = (kAlpha * raw_speed) + ((1.0f - kAlpha) * speed_cm_s);
}

float Motor::getDistance() const {
    
    noInterrupts();
    int32_t current_ticks = ticks;
    interrupts();

    return current_ticks * cm_per_tick;
}

// TODO: braking behavior depends on the driver model.
void Motor::brake() {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, HIGH);
    ledcWrite(pwm_pin, 0);
    power = 0;
}

void Motor::resetDistance() {
    noInterrupts();
    ticks = 0;
    last_ticks = 0;
    interrupts();
}

int Motor::getPower() const {
    return power;
}

float Motor::getSpeed() const {
    return speed_cm_s;
}