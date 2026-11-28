#pragma once

#include <stdint.h>

enum class FloorTile { kUnknown, kBlack, kBlue, kCheckpoint };

struct FloorSample {
    uint16_t clear, red, green, blue;
    FloorTile tile() const;
    bool usableForCalibration() const;
};

struct FloorCalibration {
    static constexpr uint8_t kTileCount = 3;
    FloorSample samples[kTileCount];
    FloorSample reserved;  // Keep the EEPROM layout compatible with older profiles.

    bool valid() const;
    FloorTile classify(const FloorSample &sample) const;
    const FloorSample &referenceAt(uint8_t index) const;
};

class FloorColor {
public:
    bool begin();
    bool read(FloorSample &sample);
    FloorTile classify(const FloorSample &sample) const;
    bool saveCalibration(const FloorCalibration &profile);
    FloorCalibration calibrationProfile() const { return calibration; }
    bool hasSavedCalibration() const { return hasProfile; }

private:
    FloorCalibration calibration{};
    bool hasProfile = false;
};

class TCS {
public:
    bool init();
    bool updateRGBC();
    void printRGB();
    char getColor();

    float red_ = 0;
    float green_ = 0;
    float blue_ = 0;
    float clear_ = 0;

private:
    FloorColor sensor_;
    FloorSample sample_{};
    bool ready_ = false;
};
