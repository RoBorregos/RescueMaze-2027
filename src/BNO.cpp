#include "BNO.h"
#include "config.h"
#include <math.h>

namespace {
constexpr uint32_t reportPeriodUs = 10000;
constexpr float degreesPerRadian = 180.0f / PI;
}

BNO::BNO() {}

void BNO::setupBNO() {
  available_ = false;
  referenceValid_ = false;
  for (uint8_t address : BnoConfig::kAddresses) {
    Wire1.beginTransmission(address);
    if (Wire1.endTransmission() != 0 || !bno_.begin_I2C(address, &Wire1))
      continue;
    if (bno_.enableReport(SH2_GAME_ROTATION_VECTOR, reportPeriodUs)) {
      available_ = true;
      referenceValid_ = true;
      bno_.wasReset();
      const uint32_t startedMs = millis();
      received_ = false;
      while (!received_ && millis() - startedMs < BnoConfig::kSetupTimeoutMs) {
        updateBNO();
        delay(2);
      }
      if (received_) return;
    }
    available_ = false;
    referenceValid_ = false;
  }
  Serial.println("BNO08x unavailable. Encoder heading enabled.");
}

void BNO::updateBNO() {
  for (uint8_t i = 0; i < 8; ++i) {
    sh2_SensorValue_t event{};
    const bool received = bno_.getSensorEvent(&event);
    if (bno_.wasReset()) {
      received_ = false;
      referenceValid_ = false;
      bno_.enableReport(SH2_GAME_ROTATION_VECTOR, reportPeriodUs);
      return;
    }
    if (!received) return;
    if (event.sensorId != SH2_GAME_ROTATION_VECTOR) continue;
    const auto &q = event.un.gameRotationVector;
    const float w = q.real, x = q.i, y = q.j, z = q.k;
    const float norm = w*w + x*x + y*y + z*z;
    if (!isfinite(norm) || norm < 0.5f || norm > 1.5f) continue;
    received_ = true;
    lastReportMs_ = millis();
    latest_ = {w, x, y, z};
    tilt_ = degreesPerRadian * asinf(constrain(2*(w*y - z*x) / norm, -1.0f, 1.0f));
  }
}

bool BNO::hasFreshReport() {
  if (!available_) return false;
  updateBNO();
  return received_ && millis() - lastReportMs_ <= BnoConfig::kReportFreshnessMs;
}

bool BNO::clockwiseHeadingOffset(const Quaternion &reference,
                                 const Quaternion &current, float &degrees) {
  // Rotate from the reference into the current attitude and keep the
  // twist around the world's vertical axis.
  const float w = current.w * reference.w + current.x * reference.x +
                  current.y * reference.y + current.z * reference.z;
  const float z = current.z * reference.w - current.w * reference.z +
                  current.y * reference.x - current.x * reference.y;
  const float twistNorm = w * w + z * z;
  if (!isfinite(twistNorm) || twistNorm < 1e-6f) return false;
  degrees = remainderf(-2.0f * degreesPerRadian * atan2f(z, w), 360.0f);
  return true;
}

bool BNO::tryReadHeading(float &heading, uint32_t *reportMs) {
  if (!hasFreshReport() || !referenceValid_ ||
      !clockwiseHeadingOffset(reference_, latest_, heading)) return false;
  if (heading < 0.0f) heading += 360.0f;
  else if (heading >= 360.0f) heading -= 360.0f;
  if (reportMs) *reportMs = lastReportMs_;
  return true;
}

float BNO::getOrientationX() {
  float heading;
  return tryReadHeading(heading) ? heading : NAN;
}

float BNO::getOrientationY() {
  return hasFreshReport() ? tilt_ - phaseCorrectionY_ : NAN;
}

void BNO::resetOrientation() {
  if (!hasFreshReport()) return;
  reference_ = latest_;
  referenceValid_ = true;
  phaseCorrectionY_ = tilt_;
}

void BNO::resetOrientationX() {
  if (!hasFreshReport()) return;
  reference_ = latest_;
  referenceValid_ = true;
}
