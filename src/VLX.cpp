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
                VLX_.startRangeContinuous(100);
  return initialized;
}

float VLX::getDistance() {

  uint16_t rawRange = VLX_.readRange();
  uint16_t status = VLX_.readRangeStatus();
  if (VLX_.timeoutOccurred()) {
    // preserve previous valid distance if timeout
    return distance;
  }

  if (status != 4) {
    distance = (float)(rawRange) / 10.0f;
    return distance;
  } else {
    rawRange = VLX_.readRange();
    status = VLX_.readRangeStatus();
    if (status != 4) {
      distance = (float)(rawRange) / 10.0f;
      return distance;
    }
  }
  return 819.0f;
}

void VLX::printDistance() {
  float printedDistance = getDistance();
  if (printedDistance != 819.0) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  } else {
    Serial.println("Out of range.");
  }
}

bool VLX::readValidMillimeters(uint16_t &millimeters) {
  if (!initialized) return false;
  const uint16_t sample = VLX_.readRange();
  if (VLX_.Status != VL53L0X_ERROR_NONE || VLX_.timeoutOccurred() ||
      VLX_.readRangeStatus() != 0 || sample == 0 || sample == 0xFFFF) return false;
  millimeters = sample;
  return true;
}

WallReading VLX::readWall(uint16_t wallDistanceMm) {
  uint16_t millimeters = 0;
  if (!readValidMillimeters(millimeters)) return WallReading::kUnknown;
  return millimeters < wallDistanceMm ? WallReading::kWall : WallReading::kOpen;
}
