#pragma once
#include "Pins_ID.h"
#include "config.h"
#include "SingleEMAFilter.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <Wire.h>

enum class WallReading : uint8_t { kUnknown, kOpen, kWall };

class VLX {
private:
  static constexpr uint32_t kTimingBudget = 33000;
  uint16_t lastRangeMm_ = 0;
  uint32_t lastRangeMs_ = 0;

public:
  bool initialized = false;
  float distance = VlxConfig::kOutOfRangeCm;
  // Movement distances are in centimeters. Wall readings use millimeters.
  Adafruit_VL53L0X VLX_ = Adafruit_VL53L0X();
  VL53L0X_RangingMeasurementData_t measure;
  float getDistance();
  void printDistance();
  VLX();
  static bool prepareBus();
  bool begin(uint8_t connector);
  bool readValidMillimeters(uint16_t &millimeters, uint32_t *sampleTimeMs = nullptr);
  WallReading readWall(uint16_t wallDistanceMm);
};
