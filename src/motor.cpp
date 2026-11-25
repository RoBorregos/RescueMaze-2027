#include "motor.h"
#include "config.h"
#include <Arduino.h>

void Motor::initialize(uint8_t in1_pin_, uint8_t in2_pin_, uint8_t pwm_pin_,
                       uint8_t enc_c1_pin_, uint8_t enc_c2_pin_,
                       bool motor_inverted_, bool encoder_inverted_, float cm_per_tick_) {
    in1_pin = in1_pin_;
    in2_pin = in2_pin_;
    pwm_pin = pwm_pin_;
    enc_c1_pin = enc_c1_pin_;
    enc_c2_pin = enc_c2_pin_;
    motor_inverted = motor_inverted_;
    encoder_inverted = encoder_inverted_;
    cm_per_tick = cm_per_tick_;

    power = 0;
    ticks = 0;
    last_ticks = 0;
    last_sample_ms = millis();
    speed_cm_s = 0.0f;

    pinMode(in1_pin, OUTPUT);
    pinMode(in2_pin, OUTPUT);
    pinMode(enc_c1_pin, INPUT);
    pinMode(enc_c2_pin, INPUT);
    analogWriteFrequency(pwm_pin, Control::kPwmFrequencyHz);
    brake();
}

void Motor::setPower(int power_) {
    power = constrain(power_, -255, 255);
    int actual_power = motor_inverted ? -power : power;

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

    analogWrite(pwm_pin, abs(actual_power));
}

// Count both edges of channel C1 and use channel C2 to determine direction.
void Motor::updateTicks() {
    if (digitalRead(enc_c1_pin) == digitalRead(enc_c2_pin)) {
        ticks++;
    } else {
        ticks--;
    }
}

int32_t Motor::readTicks() const {
    noInterrupts();
    int32_t current_ticks = ticks;
    interrupts();
    return encoder_inverted ? -current_ticks : current_ticks;
}

bool Motor::update(float dt) {
    if (dt <= 0.0f) return false;
    const uint32_t now = millis();
    const uint32_t elapsed = now - last_sample_ms;
    if (elapsed < Control::kEncoderSpeedWindowMs) return false;
    int32_t current_ticks = readTicks();
    int32_t delta_ticks = current_ticks - last_ticks;
    last_ticks = current_ticks;
    last_sample_ms = now;
    speed_cm_s = delta_ticks * cm_per_tick * 1000.0f / elapsed;
    return true;
}

float Motor::getDistance() const {
    return readTicks() * cm_per_tick;
}

// TB6612FNG: both direction pins HIGH = brake
void Motor::brake() {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, HIGH);
    analogWrite(pwm_pin, 0);
    power = 0;
}

void Motor::resetDistance() {
    noInterrupts();
    ticks = 0;
    last_ticks = 0;
    interrupts();
    last_sample_ms = millis();
    speed_cm_s = 0.0f;
}

int32_t Motor::getTicks() const { return readTicks(); }

float Motor::getSingleEdgeTicksPerWindow() const {
    // Convert the two-edge count to the units expected by the 2026 wheel PID.
    return speed_cm_s * (Control::kEncoderSpeedWindowMs / 1000.0f) / (2.0f * cm_per_tick);
}

int Motor::getPower() const {
    return power;
}

float Motor::getSpeed() const {
    return speed_cm_s;
}
