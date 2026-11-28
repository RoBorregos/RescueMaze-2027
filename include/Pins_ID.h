#pragma once
#include <Arduino.h>

namespace MotorID {
constexpr uint8_t M1 = 0;
constexpr uint8_t M2 = 1;
constexpr uint8_t M3 = 2;
constexpr uint8_t M4 = 3;
constexpr uint8_t kCount = 4;

// Assign each wheel to the connector printed on the board.
constexpr uint8_t MotoFL = M1;
constexpr uint8_t MotoFR = M2;
constexpr uint8_t MotoBL = M4;
constexpr uint8_t MotoBR = M3;

static_assert(MotoFL < kCount && MotoFR < kCount && MotoBL < kCount && MotoBR < kCount &&
                 MotoFL != MotoFR && MotoFL != MotoBL && MotoFL != MotoBR && MotoFR != MotoBL && MotoFR != MotoBR &&
                 MotoBL != MotoBR,
              "Assign M1-M4 exactly once across MotoFL, MotoFR, MotoBL and MotoBR.");

constexpr uint8_t kBackRight = 3;
constexpr uint8_t kBackLeft = 2;
constexpr uint8_t kFrontRight = 1;
constexpr uint8_t kFrontLeft = 0;
} // namespace MotorID

namespace MotorPins {
// These arrays stay in connector order: M1, M2, M3, M4.
constexpr uint8_t encoder[MotorID::kCount] = {4, 20, 30, 32}; // Interrupt on channel A rising edge.
constexpr uint8_t encoderC2[MotorID::kCount] = {5, 15, 31, 33}; // Channel B determines direction.
constexpr uint8_t pwmPin[MotorID::kCount] = {6, 11, 12, 28};
constexpr uint8_t digitalOne[MotorID::kCount] = {7, 10, 25, 27};
constexpr uint8_t digitalTwo[MotorID::kCount] = {8, 9, 24, 26};
constexpr bool motorInverted[MotorID::kCount] = {false, false, false, true};
// Forward should count positive. Flip the encoder for that connector if needed.
constexpr bool kEncoderInverted[MotorID::kCount] = {true, false, false, false};
} // namespace MotorPins

namespace Pins {
// Motor and PID slots are ordered FL, FR, BL, BR.
constexpr uint8_t encoder[MotorID::kCount] = {
    MotorPins::encoder[MotorID::MotoFL], MotorPins::encoder[MotorID::MotoFR],
    MotorPins::encoder[MotorID::MotoBL], MotorPins::encoder[MotorID::MotoBR],
};
constexpr uint8_t encoderC2[MotorID::kCount] = {
    MotorPins::encoderC2[MotorID::MotoFL], MotorPins::encoderC2[MotorID::MotoFR],
    MotorPins::encoderC2[MotorID::MotoBL], MotorPins::encoderC2[MotorID::MotoBR],
};
constexpr bool encoderInverted[MotorID::kCount] = {
    MotorPins::kEncoderInverted[MotorID::MotoFL], MotorPins::kEncoderInverted[MotorID::MotoFR],
    MotorPins::kEncoderInverted[MotorID::MotoBL], MotorPins::kEncoderInverted[MotorID::MotoBR],
};
constexpr uint8_t pwmPin[MotorID::kCount] = {
    MotorPins::pwmPin[MotorID::MotoFL], MotorPins::pwmPin[MotorID::MotoFR],
    MotorPins::pwmPin[MotorID::MotoBL], MotorPins::pwmPin[MotorID::MotoBR],
};
constexpr uint8_t digitalOne[MotorID::kCount] = {
    MotorPins::digitalOne[MotorID::MotoFL], MotorPins::digitalOne[MotorID::MotoFR],
    MotorPins::digitalOne[MotorID::MotoBL], MotorPins::digitalOne[MotorID::MotoBR],
};
constexpr uint8_t digitalTwo[MotorID::kCount] = {
    MotorPins::digitalTwo[MotorID::MotoFL], MotorPins::digitalTwo[MotorID::MotoFR],
    MotorPins::digitalTwo[MotorID::MotoBL], MotorPins::digitalTwo[MotorID::MotoBR],
};
constexpr bool motorInverted[MotorID::kCount] = {
    MotorPins::motorInverted[MotorID::MotoFL], MotorPins::motorInverted[MotorID::MotoFR],
    MotorPins::motorInverted[MotorID::MotoBL], MotorPins::motorInverted[MotorID::MotoBR],
};

constexpr uint8_t vlxXshut[8] = {0, 1, 2, 23, 22, 40, 39, 38};
// Physical connector VLX5 has no XSHUT, regardless of its position.
constexpr uint8_t vlxWithoutXshut = 5;
constexpr uint8_t i2cSda = 18;
constexpr uint8_t i2cScl = 19;
constexpr uint8_t i2c1Sda = 17;
constexpr uint8_t i2c1Scl = 16;
// Leave optional peripherals unassigned until the wiring is confirmed.
constexpr int kUnassignedPin = -1;
constexpr int leftServoPin = kUnassignedPin;
constexpr int rightServoPin = kUnassignedPin;
constexpr int checkpointPin = kUnassignedPin;
constexpr int ledsPin = kUnassignedPin;
constexpr int leftLimitSwitchPin = kUnassignedPin;
constexpr int rightLimitSwitchPin = kUnassignedPin;

constexpr bool isAuxPinFree(int pin) {
  if (pin == kUnassignedPin) return true;
  if (pin < 0 || pin >= CORE_NUM_DIGITAL) return false;
  if (pin == i2cSda || pin == i2cScl || pin == i2c1Sda || pin == i2c1Scl) return false;
  for (uint8_t xshut : vlxXshut) {
    if (pin == xshut) return false;
  }
  for (uint8_t i = 0; i < MotorID::kCount; ++i) {
    if (pin == encoder[i] || pin == encoderC2[i] || pin == pwmPin[i] ||
        pin == digitalOne[i] || pin == digitalTwo[i]) return false;
  }
  return true;
}
constexpr bool validAuxPins() {
  constexpr int auxPins[] = {leftServoPin, rightServoPin, checkpointPin, ledsPin,
                             leftLimitSwitchPin, rightLimitSwitchPin};
  for (unsigned i = 0; i < sizeof(auxPins) / sizeof(auxPins[0]); ++i) {
    if (auxPins[i] == kUnassignedPin) continue;
    if (!isAuxPinFree(auxPins[i])) return false;
    for (unsigned j = 0; j < i; ++j) {
      if (auxPins[i] == auxPins[j]) return false;
    }
  }
  return true;
}
static_assert(validAuxPins(),
              "Auxiliary pins must be distinct and must not overlap motor, encoder, I2C or VLX pins.");

inline bool checkpointActive() {
  return checkpointPin != kUnassignedPin && digitalRead(checkpointPin) == HIGH;
}

} // namespace Pins
namespace LimitSwitchID {
constexpr uint8_t kLeft = 0;
constexpr uint8_t kRight = 1;
} // namespace LimitSwitchID

namespace vlxID {
constexpr uint8_t VLX1 = 1;
constexpr uint8_t VLX2 = 2;
constexpr uint8_t VLX3 = 3;
constexpr uint8_t VLX4 = 4;
constexpr uint8_t VLX5 = 5;
constexpr uint8_t VLX6 = 6;
constexpr uint8_t VLX7 = 7;
constexpr uint8_t VLX8 = 8;
constexpr uint8_t kNone = 0;

// Editable: assign each position to the connector printed on the board.
constexpr uint8_t FL = VLX3;  // Front left
constexpr uint8_t FR = VLX1;  // Front right
constexpr uint8_t RF = VLX8;  // Right side, front
constexpr uint8_t RB = kNone; // Right side, back
constexpr uint8_t LF = VLX5;  // Left side, front
constexpr uint8_t LB = kNone; // Left side, back
constexpr uint8_t BL = VLX7;  // Back left: retains the previous rear connector
constexpr uint8_t BR = kNone; // Back right

// Sensor slots used by wall detection and movement.
constexpr uint8_t frontLeft = 3;
constexpr uint8_t frontRight = 0;
constexpr uint8_t rightFront = 4;
constexpr uint8_t rightBack = 5;
constexpr uint8_t leftFront = 2;
constexpr uint8_t leftBack = 6;
constexpr uint8_t backLeft = 1;
constexpr uint8_t backRight = 7;
constexpr uint8_t connectors[] = {FR, BL, LF, FL, RF, RB, LB, BR};

constexpr bool validConnectorMap() {
  uint16_t used = 0;
  for (uint8_t connector : connectors) {
    if (connector == kNone) continue;
    if (connector < VLX1 || connector > VLX8) return false;
    const uint16_t bit = 1u << (connector - 1);
    if (used & bit) return false;
    used |= bit;
  }
  return true;
}
static_assert(validConnectorMap(), "Assign each installed VLX a distinct connector VLX1-VLX8.");
static_assert(FL && FR && RF && LF && BL,
              "navigation needs FL, FR, RF, LF and BL assigned.");
} // namespace vlxID
