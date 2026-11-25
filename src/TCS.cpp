#include "TCS.h"

#include <Arduino.h>
#include <Wire.h>

#include "CalibrationStorage.h"

namespace {
constexpr uint8_t kAddress = 0x29;
constexpr uint8_t kCommand = 0x80;
constexpr uint8_t kAutoIncrement = 0x20;
constexpr int kColorTolerance = 20;
constexpr uint32_t kMinimumDistanceSquared = 16 * 16;
constexpr uint8_t kIntegration = 0xEB;
constexpr uint16_t kSaturation = (256 - kIntegration) * 1024;
constexpr uint32_t kProfileVersion = 0x54435301;
constexpr FloorSample kReferenceColors[FloorCalibration::kTileCount] = {
    {0, 32, 25, 18}, {0, 62, 72, 82}, {0, 1540, 1354, 1272},
};

bool write(uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(kAddress);
    Wire1.write(kCommand | reg);
    Wire1.write(value);
    return Wire1.endTransmission() == 0;
}

bool readRegisters(uint8_t command, uint8_t *out, uint8_t count) {
    Wire1.beginTransmission(kAddress);
    Wire1.write(kCommand | command);
    if (Wire1.endTransmission(false) != 0 || Wire1.requestFrom(kAddress, count) != count) return false;
    for (uint8_t i = 0; i < count; ++i) out[i] = Wire1.read();
    return true;
}

uint16_t channel(const uint8_t *data) {
    return data[0] | (uint16_t(data[1]) << 8);
}

bool near(uint16_t value, int reference) {
    return abs(int(value) - reference) < kColorTolerance;
}

uint32_t colorDistanceSquared(const FloorSample &first, const FloorSample &second) {
    const int32_t red = int32_t(first.red) - second.red;
    const int32_t green = int32_t(first.green) - second.green;
    const int32_t blue = int32_t(first.blue) - second.blue;
    return red * red + green * green + blue * blue;
}
}

bool FloorColor::begin() {
    hasProfile = false;
    uint8_t id = 0;
    if (!readRegisters(0x12, &id, 1) || (id != 0x44 && id != 0x4D)) return false;
    if (!write(0x01, kIntegration) || !write(0x0F, 0x01) || !write(0x00, 0x01)) return false;
    delay(3);
    if (!write(0x00, 0x03)) return false;
    delay(55);
    hasProfile = CalibrationStorage::load(CalibrationStorage::kTcsAddress, kProfileVersion, calibration) &&
                 calibration.valid();
    if (!hasProfile) calibration = {};
    return true;
}

bool FloorColor::read(FloorSample &sample) {
    uint8_t data[8];
    uint8_t status = 0;
    if (!readRegisters(0x13, &status, 1) || !(status & 1) ||
        !readRegisters(kAutoIncrement | 0x14, data, sizeof(data))) return false;
    sample = {channel(data), channel(data + 2), channel(data + 4), channel(data + 6)};
    return true;
}

FloorTile FloorSample::tile() const {
    return FloorCalibration{}.classify(*this);
}

bool FloorSample::usableForCalibration() const {
    return clear != 0 && clear < kSaturation && red < kSaturation && green < kSaturation && blue < kSaturation;
}

bool FloorCalibration::valid() const {
    for (uint8_t i = 0; i < kTileCount; ++i) {
        const FloorSample &candidate = samples[i];
        // An empty slot retains the reference values from the 2026 code.
        if ((candidate.clear || candidate.red || candidate.green || candidate.blue) &&
            !candidate.usableForCalibration()) return false;
        for (uint8_t j = 0; j < i; ++j) {
            if (colorDistanceSquared(referenceAt(i), referenceAt(j)) < kMinimumDistanceSquared) return false;
        }
    }
    return true;
}

const FloorSample &FloorCalibration::referenceAt(uint8_t index) const {
    return samples[index].clear != 0 ? samples[index] : kReferenceColors[index];
}

FloorTile FloorCalibration::classify(const FloorSample &sample) const {
    FloorTile closest = FloorTile::kUnknown;
    uint32_t shortest = UINT32_MAX;
    for (uint8_t i = 0; i < kTileCount; ++i) {
        const FloorSample &reference = referenceAt(i);
        if (!near(sample.red, reference.red) || !near(sample.green, reference.green) ||
            !near(sample.blue, reference.blue)) continue;
        const uint32_t distance = colorDistanceSquared(sample, reference);
        if (distance < shortest) {
            shortest = distance;
            closest = static_cast<FloorTile>(i + 1);
        } else if (distance == shortest) {
            closest = FloorTile::kUnknown;
        }
    }
    return closest;
}

FloorTile FloorColor::classify(const FloorSample &sample) const {
    return calibration.classify(sample);
}

bool FloorColor::saveCalibration(const FloorCalibration &profile) {
    if (!profile.valid() || !CalibrationStorage::save(CalibrationStorage::kTcsAddress, kProfileVersion, profile)) {
        return false;
    }
    calibration = profile;
    hasProfile = true;
    return true;
}


bool TCS::init() {
    ready_ = sensor_.begin();
    if (!ready_) Serial.println("TCS34725 not found; floor color detection disabled.");
    else if (sensor_.hasSavedCalibration()) Serial.println("TCS profile loaded from EEPROM.");
    else Serial.println("TCS using default references; calibrate before running the maze.");
    return ready_;
}

bool TCS::updateRGBC() {
    if (!ready_ || !sensor_.read(sample_) || !sample_.usableForCalibration()) {
        sample_ = {};
        red_ = green_ = blue_ = clear_ = 0;
        return false;
    }
    red_ = sample_.red;
    green_ = sample_.green;
    blue_ = sample_.blue;
    clear_ = sample_.clear;
    return true;
}

void TCS::printRGB() {
    if (!updateRGBC()) return;
    Serial.println(red_);
    Serial.println(green_);
    Serial.println(blue_);
}

char TCS::getColor() {
    if (!updateRGBC()) return 'U';
    switch (sensor_.classify(sample_)) {
      case FloorTile::kBlack: return 'N';
      case FloorTile::kBlue: return 'B';
      case FloorTile::kCheckpoint: return 'C';
      default: return 'U';
    }
}
