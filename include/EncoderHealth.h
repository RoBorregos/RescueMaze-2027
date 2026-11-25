#ifndef ENCODER_HEALTH_H
#define ENCODER_HEALTH_H

#include "Pins_ID.h"
#include "config.h"

class EncoderHealth {
private:
  struct Wheel {
    uint32_t lastTicks = 0;
    uint32_t lastActivityMs = 0;
    bool watching = false;
    bool failed = false;
  };
  Wheel wheels_[MotorID::kCount];

  template <typename Sample>
  float sideAverage(const Sample *values, uint8_t front, uint8_t back) const {
    if (failed(front)) return failed(back) ? 0 : values[back];
    if (failed(back)) return values[front];
    return (values[front] + values[back]) / 2.0f;
  }

public:
  static uint8_t partner(uint8_t wheel) {
    constexpr uint8_t partners[] = {
        MotorID::kBackLeft, MotorID::kBackRight,
        MotorID::kFrontLeft, MotorID::kFrontRight,
    };
    return partners[wheel];
  }

  // Returns true only on the transition to failed. Restart to clear a fault.
  bool observe(uint8_t wheel, uint32_t ticks, uint16_t pwm, uint32_t now) {
    Wheel &state = wheels_[wheel];
    if (state.failed) return false;
    if (pwm < EncoderConfig::kWatchPwm) {
      state.watching = false;
    } else if (!state.watching || ticks != state.lastTicks) {
      state.watching = true;
      state.lastActivityMs = now;
    } else if (now - state.lastActivityMs >= EncoderConfig::kNoPulsesMs) {
      state.failed = true;
      return true;
    }
    state.lastTicks = ticks;
    return false;
  }

  bool failed(uint8_t wheel) const { return wheels_[wheel].failed; }

  bool hasFeedbackOnBothSides() const {
    return !(failed(MotorID::kFrontLeft) && failed(MotorID::kBackLeft)) &&
           !(failed(MotorID::kFrontRight) && failed(MotorID::kBackRight));
  }

  template <typename Sample>
  float average(const Sample *values) const {
    return (sideAverage(values, MotorID::kFrontLeft, MotorID::kBackLeft) +
            sideAverage(values, MotorID::kFrontRight, MotorID::kBackRight)) / 2.0f;
  }

  void endMotion() {
    for (Wheel &wheel : wheels_) wheel.watching = false;
  }
};

#endif
