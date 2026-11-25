#include "BNO.H"
#include "Pins_ID.h"
#include <math.h>

float z_rotation;
float angle;

namespace {
constexpr uint32_t reportPeriodUs = 10000;
constexpr float degreesPerRadian = 180.0f / PI;
}

BNO::BNO() {}

void BNO::setupBNO() {
  for (uint8_t address : {0x4A, 0x4B}) {
    Wire1.beginTransmission(address);
    if (Wire1.endTransmission() != 0 || !bno_.begin_I2C(address, &Wire1))
      continue;
    if (bno_.enableReport(SH2_GAME_ROTATION_VECTOR, reportPeriodUs)) {
      bno_.wasReset();
      const uint32_t startedMs = millis();
      received_ = false;
      while (!received_ && millis() - startedMs < 1000) {
        updateBNO();
        delay(2);
      }
      if (received_) return;
      break;
    }
  }
  Serial.println("BNO08x not found on Wire1.");
  while (true) delay(50);
}

void BNO::updateBNO() {
  for (uint8_t i = 0; i < 8; ++i) {
    sh2_SensorValue_t event{};
    const bool received = bno_.getSensorEvent(&event);
    if (bno_.wasReset()) {
      received_ = false;
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

void BNO::requireFreshReport() {
  updateBNO();
  if (received_ && millis() - lastReportMs_ <= 250) return;
  stopOnOrientationFault("Stopped: no recent BNO08x report.");
}

void BNO::stopOnOrientationFault(const char *message) {
  for (uint8_t pin : Pins::pwmPin) analogWrite(pin, 0);
  Serial.println(message);
  while (true) delay(50);
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

float BNO::getOrientationX() {
  requireFreshReport();
  float heading;
  if (!clockwiseHeadingOffset(reference_, latest_, heading)) {
    stopOnOrientationFault("Stopped: BNO08x heading is undefined.");
  }
  angle = heading;
  if (angle < 0) angle += 360;
  else if (angle >= 360) angle -= 360;
  z_rotation = angle > 180 ? angle - 360.0f : angle;
  return angle;
}

float BNO::getOrientationY() {
  requireFreshReport();
  return tilt_ - phaseCorrectionY_;
}

void BNO::resetOrientation() {
  resetOrientationX();
  phaseCorrectionY_ = tilt_;
}

void BNO::resetOrientationX() {
  requireFreshReport();
  reference_ = latest_;
}
