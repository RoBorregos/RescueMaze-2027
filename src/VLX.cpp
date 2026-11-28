#include "VLX.h"
#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr uint8_t kDefaultAddress = 0x29;
constexpr uint8_t kFirstAssignedAddress = 0x30;
constexpr uint8_t kStartupAddress = 0x38;
constexpr uint8_t kAddressRegister = 0x8A;
constexpr uint8_t kSoftResetRegister = 0xBF;
constexpr uint8_t kAddressNack = 2;
uint8_t probeAddress(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission();
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool moveAddress(uint8_t from, uint8_t to) {
    if (probeAddress(to) != kAddressNack || !writeRegister(from, kAddressRegister, to)) return false;
    delay(10);
    return probeAddress(to) == 0 && probeAddress(from) == kAddressNack;
}

bool resetAtDefaultAddress() {
    if (!writeRegister(kDefaultAddress, kSoftResetRegister, 0)) return false;
    delay(2);
    if (!writeRegister(kDefaultAddress, kSoftResetRegister, 1)) return false;
    delay(15);
    return true;
}

bool stageUnswitchedSensor() {
    uint8_t found = 0;
    // A Teensy reset can leave the powered sensor at a previously assigned address.
    for (uint8_t address = kDefaultAddress; address <= kStartupAddress;
         address = address == kDefaultAddress ? kFirstAssignedAddress : address + 1) {
        const uint8_t result = probeAddress(address);
        if (result == kAddressNack) continue;
        if (result != 0 || found != 0) return false;
        found = address;
    }
    return found == 0 || found == kStartupAddress || moveAddress(found, kStartupAddress);
}

}

VLX::VLX() {}

bool VLX::prepareBus() {
  for (uint8_t i = 0; i < sizeof(Pins::vlxXshut); ++i) {
    if (i + 1 == Pins::vlxWithoutXshut) continue;
    digitalWrite(Pins::vlxXshut[i], LOW);
    pinMode(Pins::vlxXshut[i], OUTPUT);
  }
  delay(10);
  return stageUnswitchedSensor();
}

bool VLX::begin(uint8_t connector) {
  initialized = false;
  lastRangeMm_ = 0;
  distance = VlxConfig::kOutOfRangeCm;
  if (connector < 1 || connector > sizeof(Pins::vlxXshut)) return false;
  const bool hasXshut = connector != Pins::vlxWithoutXshut;
  const uint8_t address = hasXshut ? kFirstAssignedAddress + connector - 1 : kDefaultAddress;
  if (hasXshut) {
    pinMode(Pins::vlxXshut[connector - 1], INPUT);
    delay(15);
  } else if (probeAddress(kStartupAddress) == 0 &&
             (!moveAddress(kStartupAddress, kDefaultAddress) || !resetAtDefaultAddress())) {
    return false;
  }
  initialized = probeAddress(kDefaultAddress) == 0 &&
                VLX_.begin(address, false, &Wire) &&
                VLX_.setMeasurementTimingBudgetMicroSeconds(kTimingBudget) &&
                VLX_.startRangeContinuous(VlxConfig::kRangePeriodMs);
  return initialized;
}

float VLX::getDistance() {
  uint16_t millimeters;
  distance = readValidMillimeters(millimeters) ? millimeters / 10.0f : VlxConfig::kOutOfRangeCm;
  return distance;
}

void VLX::printDistance() {
  const float centimeters = getDistance();
  if (centimeters != VlxConfig::kOutOfRangeCm) {
    Serial.print("Distance: ");
    Serial.print(centimeters);
    Serial.println(" cm");
  } else {
    Serial.println("No valid range available.");
  }
}

bool VLX::readValidMillimeters(uint16_t &millimeters, uint32_t *sampleTimeMs) {
  if (!initialized) return false;
  if (VLX_.isRangeComplete()) {
    lastRangeMm_ = 0;
    if (VLX_.Status != VL53L0X_ERROR_NONE) return false;
    const uint16_t sample = VLX_.readRangeResult();
    if (VLX_.Status != VL53L0X_ERROR_NONE || VLX_.readRangeStatus() != 0 ||
        sample == 0 || sample == 0xFFFF) return false;
    lastRangeMm_ = sample;
    lastRangeMs_ = millis();
  }
  // Reuse the last valid sample between continuous measurements.
  if (lastRangeMm_ == 0 || millis() - lastRangeMs_ > VlxConfig::kRangeFreshnessMs)
    return false;
  millimeters = lastRangeMm_;
  if (sampleTimeMs) *sampleTimeMs = lastRangeMs_;
  return true;
}

WallReading VLX::readWall(uint16_t wallDistanceMm) {
  uint16_t millimeters = 0;
  if (!readValidMillimeters(millimeters)) return WallReading::kUnknown;
  return millimeters < wallDistanceMm ? WallReading::kWall : WallReading::kOpen;
}
