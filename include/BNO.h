#ifndef BNO_H
#define BNO_H

#include <Adafruit_BNO08x.h>

struct OrientationSample {
    float heading;
    float roll;
    float pitch;
};

struct OrientationQuaternion {
    float w;
    float x;
    float y;
    float z;
};

class Orientation {
public:
    bool begin();
    bool read(OrientationSample &sample);
    bool readQuaternion(OrientationQuaternion &sample);
    bool startCalibration();
    bool calibrationStatus(uint8_t &accuracy);
    bool saveCalibration();

private:
    Adafruit_BNO08x bno08x{-1};
    bool usingBno08x = false;
    bool haveSample = false;
    uint32_t lastSample = 0;
    OrientationQuaternion latest{};
    uint8_t reportAccuracy = 0;
    bool calibrating = false;
    uint32_t accuracySince = 0;
};

float angleError(float target, float measured);
bool clockwiseHeadingOffset(const OrientationQuaternion &reference,
                            const OrientationQuaternion &current, float &degrees);

#endif
