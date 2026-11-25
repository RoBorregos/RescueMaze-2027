#include "motor.h"
#include "config.h"

void Motor::initialize(uint8_t in_1, uint8_t in_2, uint8_t en,
                      uint8_t numMotor) {
  in1 = in_1;
  in2 = in_2;
  enable = en;
  inverted = Pins::motorInverted[numMotor];
  encoderC2Pin = Pins::encoderC2[numMotor];
  encoderInverted = Pins::encoderInverted[numMotor];
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enable, OUTPUT);
  pinMode(Pins::encoder[numMotor], INPUT);
  pinMode(encoderC2Pin, INPUT);
  analogWriteFrequency(enable, Control::kPwmFrequencyHz);
  analogWrite(enable, 0);
}

Motor::Motor() {
  // default constructor
}
void Motor::updateTics() {
  ++totalEncoderTics_;
  // Channel A is high here, so channel B high is positive before inversion.
  const bool channelBHigh = digitalRead(encoderC2Pin) == HIGH;
  const int step = (channelBHigh != encoderInverted) ? 1 : -1;
  tics += step;
  deltaTics += step;
  const uint32_t now = millis();
  const uint32_t elapsed = now - last_time;
  if (elapsed >= Control::kEncoderSpeedWindowMs) {
    ticsSpeed = deltaTics * static_cast<float>(Control::kEncoderSpeedWindowMs) / elapsed;
    deltaTics = 0;
    last_time = now;
  }
}
void Motor::resetTics() {
  tics = 0;
  deltaTics = 0;
}
int Motor::getTics() { return tics; }
void Motor::setSpeed(uint16_t velocity) {
  speed = velocity;
  speed = constrain(speed, 0, static_cast<int>(Control::kPwmMax));
  analogWrite(enable, speed);
  delay(1); // wait one milli (dont delate)
}
void Motor::ahead() {
  digitalWrite(in1, inverted ? LOW : HIGH);
  digitalWrite(in2, inverted ? HIGH : LOW);
}
void Motor::back() {
  digitalWrite(in1, inverted ? HIGH : LOW);
  digitalWrite(in2, inverted ? LOW : HIGH);
}
void Motor::stop() {
  digitalWrite(in1, 1);
  digitalWrite(in2, 1);
}
double Motor::getSpeed() { return speed; }
uint32_t Motor::getTotalEncoderTics() const { return totalEncoderTics_; }
float Motor::getTicsSpeed() {
  return millis() - last_time <= 2 * Control::kEncoderSpeedWindowMs ? ticsSpeed : 0;
}
