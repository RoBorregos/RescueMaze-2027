#pragma once
#include "Pins_ID.h"
#include <Arduino.h>

class Motor {
private:
  uint8_t in1;
  uint8_t in2;
  uint8_t enable;
  bool inverted = false;
  uint8_t encoderC2Pin;
  bool encoderInverted = false;
  uint16_t speed;
  volatile float ticsSpeed = 0;
  volatile unsigned long last_time = 0;
  volatile uint32_t totalEncoderTics_ = 0;

public:
  volatile int tics;
  volatile int deltaTics;
  void initialize(uint8_t, uint8_t, uint8_t, uint8_t);
  Motor();
  void updateTics();
  void setSpeed(uint16_t);
  void ahead();
  void back();
  void stop();
  double getSpeed();
  void resetTics();
  int getTics();
  uint32_t getTotalEncoderTics() const;
  float getTicsSpeed();
};
