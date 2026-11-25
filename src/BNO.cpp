#include "BNO.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.h"

namespace {
constexpr uint32_t kReportPeriodUs = 10000;
constexpr uint32_t kMaximumAgeMs = 120;
constexpr uint32_t kDcdUpdatePeriodMs = 5000;
constexpr float kDegreesPerRadian = 180.0f / Geometry::kPi;
constexpr float kHeadingPolarity = MazeConfig::kInvertBnoHeading ? -1.0f : 1.0f;

float normSquared(const OrientationQuaternion &q) {
    return q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
}

bool validQuaternion(const OrientationQuaternion &q) {
    const float norm = normSquared(q);
    return isfinite(norm) && norm >= 0.5f && norm <= 1.5f;
}
}

float angleError(float target, float measured) {
    return remainderf(target - measured, 360.0f);
}

bool clockwiseHeadingOffset(const OrientationQuaternion &reference,
                            const OrientationQuaternion &current, float &degrees) {
    // Relative rotation in the world frame: current * conjugate(reference).
    // Project its twist onto world vertical using just the scalar and Z terms.
    const float w = current.w*reference.w + current.x*reference.x +
                    current.y*reference.y + current.z*reference.z;
    const float z = current.z*reference.w - current.w*reference.z +
                    current.y*reference.x - current.x*reference.y;
    const float twistNorm = w*w + z*z;
    if (!isfinite(twistNorm) || twistNorm < 1e-6f) return false;
    degrees = remainderf(-kHeadingPolarity * 2.0f * kDegreesPerRadian * atan2f(z, w), 360.0f);
    return true;
}

bool Orientation::begin() {
    usingBno08x = false;
    calibrating = false;
    haveSample = false;
    const uint8_t bno08xAddresses[] = {0x4A, 0x4B};
    for (uint8_t address : bno08xAddresses) {
        Wire1.beginTransmission(address);
        if (Wire1.endTransmission() != 0 || !bno08x.begin_I2C(address, &Wire1)) continue;
        usingBno08x = bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, kReportPeriodUs);
        if (usingBno08x) {
            bno08x.wasReset();
            Serial.printf("BNO08x on Wire1 at address 0x%02X.\n", unsigned(address));
            return true;
        }
    }
    return false;
}

bool Orientation::startCalibration() {
    if (!usingBno08x || sh2_setDcdAutoSave(false) != SH2_OK ||
        sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO) != SH2_OK) return false;
    haveSample = false;
    calibrating = true;
    return true;
}

bool Orientation::calibrationStatus(uint8_t &accuracy) {
    OrientationQuaternion sample{};
    if (!readQuaternion(sample)) return false;
    accuracy = reportAccuracy;
    return true;
}

bool Orientation::saveCalibration() {
    if (!calibrating) return false;
    uint8_t accuracy = 0;
    if (!calibrationStatus(accuracy) || !calibrating) return false;
    if (accuracy != 3 || millis() - accuracySince < kDcdUpdatePeriodMs) return false;
    calibrating = sh2_saveDcdNow() != SH2_OK;
    return !calibrating;
}

bool Orientation::read(OrientationSample &sample) {
    OrientationQuaternion q{};
    if (!readQuaternion(q)) return false;
    const float w = q.w, x = q.x, y = q.y, z = q.z;
    const float norm = normSquared(q);
    sample = {
        -kHeadingPolarity * kDegreesPerRadian * atan2f(2 * (w*z + x*y), norm - 2 * (y*y + z*z)),
        kDegreesPerRadian * atan2f(2 * (w*x + y*z), norm - 2 * (x*x + y*y)),
        kDegreesPerRadian * asinf(constrain(2 * (w*y - z*x) / norm, -1.0f, 1.0f)),
    };
    return true;
}

bool Orientation::readQuaternion(OrientationQuaternion &sample) {
    if (!usingBno08x) return false;

    // Keep the newest queued report, including after an idle period.
    for (uint8_t i = 0; i < 8; ++i) {
        sh2_SensorValue_t event{};
        const bool received = bno08x.getSensorEvent(&event);
        if (bno08x.wasReset()) {
            haveSample = false;
            calibrating = false;
            usingBno08x = bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, kReportPeriodUs);
            return false;
        }
        if (!received) break;
        if (event.sensorId != SH2_GAME_ROTATION_VECTOR) continue;
        const auto &q = event.un.gameRotationVector;
        latest = {q.real, q.i, q.j, q.k};
        if (!validQuaternion(latest)) {
            haveSample = false;
            return false;
        }
        const uint8_t accuracy = event.status & 3;
        const uint32_t receivedAt = millis();
        if (!haveSample || accuracy != reportAccuracy || receivedAt - lastSample > 2 * kMaximumAgeMs) {
            accuracySince = receivedAt;
        }
        lastSample = receivedAt;
        reportAccuracy = accuracy;
        haveSample = true;
    }
    if (!haveSample || millis() - lastSample > kMaximumAgeMs) return false;
    sample = latest;
    return true;
}
