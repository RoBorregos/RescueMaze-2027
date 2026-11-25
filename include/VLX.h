#ifndef RESCUE_VLX_H
#define RESCUE_VLX_H

#include <Adafruit_VL53L0X.h>

#include "config.h"

struct VlxSample {
    uint16_t raw[3];
    uint16_t millimeters;
};

enum class VlxReading { kUnknown, kDistance, kNoReturn };

class VLX {
public:
    static constexpr float kDistanceToWall = MazeConfig::kSideWallDistanceMm / 10.0f;
    static constexpr float kOutOfRange = 819.0f;

    VLX() = default;
    explicit VLX(uint8_t index);
    static bool prepareBus();
    bool begin();
    void setSensorIndex(uint8_t index);
    void updateDistance();
    float getDistance();  // Centimeters, as in the 2026 controller.
    bool readMillimeters(uint16_t &millimeters);
    void startNavigationRead();
    bool pollNavigationRead(VlxReading &reading, uint16_t &millimeters);
    bool readSample(VlxSample &sample);
    bool readReadyMillimeters(uint16_t &millimeters);
    void printDistance();
    bool isWall();
    bool isWall(bool &wall);

    bool initialized = false;
    float distance = 0.0f;

private:
    struct SampleWindow {
        VlxSample sample{};
        uint32_t waitingSince = 0;
        uint8_t attempts = 0;
        uint8_t validCount = 0;
        uint8_t noReturnSamples = 0;
    };

    void startWindow(SampleWindow &window);
    bool pollWindow(SampleWindow &window, uint8_t maxSamples, uint8_t requiredValid);
    bool readRangeResult(uint16_t &millimeters);
    uint16_t wallThresholdMm() const;
    Adafruit_VL53L0X sensor;
    SampleWindow navigationWindow{};
    bool navigationPending = false;
    uint8_t index = 0;
    uint16_t lastMillimeters = 0;
    bool valid = false;
};

#endif
