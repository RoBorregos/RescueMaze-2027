#pragma once

#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
class BNO {
private:
  Adafruit_BNO08x bno_{-1};
  struct Quaternion {
    float w, x, y, z;
  };
  Quaternion latest_{1, 0, 0, 0};
  Quaternion reference_{1, 0, 0, 0};
  static bool clockwiseHeadingOffset(const Quaternion &reference,
                                     const Quaternion &current, float &degrees);
  bool available_ = false;
  bool referenceValid_ = false;
  float tilt_ = 0.0f;
  bool received_ = false;
  uint32_t lastReportMs_ = 0;
  bool hasFreshReport();
  void updateBNO();
  float phaseCorrectionY_ = 0.0;

public:
  float angle_initial;
  BNO();
  void setupBNO();
  bool tryReadHeading(float &heading, uint32_t *reportMs = nullptr);
  float getOrientationX();
  float getOrientationY();
  void resetOrientation();
  void resetOrientationX();
};
