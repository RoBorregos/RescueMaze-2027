#ifndef VLX_H
#define VLX_H
#include "Pins_ID.h"
#include "SingleEMAFilter.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <Wire.h>

enum class WallReading : uint8_t { kUnknown, kOpen, kWall };

class VLX {
private:
  static constexpr uint32_t kTimingBudget = 33000;

public:
  static constexpr uint8_t kDistanceToWall = 15;
  bool initialized = false;
  float distance = 0.0f;
  Adafruit_VL53L0X VLX_ = Adafruit_VL53L0X();
  VL53L0X_RangingMeasurementData_t measure;
  float getDistance();
  void printDistance();
  VLX();
  static bool prepareBus();
  bool begin(uint8_t connector);
  bool readValidMillimeters(uint16_t &millimeters);
  WallReading readWall(uint16_t wallDistanceMm);
};
#endif
