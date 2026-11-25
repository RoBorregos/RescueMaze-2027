#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

namespace MotorID {
constexpr uint8_t M1 = 0;
constexpr uint8_t M2 = 1;
constexpr uint8_t M3 = 2;
constexpr uint8_t M4 = 3;
constexpr uint8_t kCount = 4;

// Physical position -> motor connector. The pin arrays stay in M1–M4 order.
constexpr uint8_t FL = M1;
constexpr uint8_t FR = M2;
constexpr uint8_t BL = M4;
constexpr uint8_t BR = M3;
static_assert(FL < kCount && FR < kCount && BL < kCount && BR < kCount &&
              (1u << FL | 1u << FR | 1u << BL | 1u << BR) == 0x0F,
              "Assign each of M1–M4 to exactly one wheel position");
constexpr uint8_t kFrontLeft = FL;
constexpr uint8_t kFrontRight = FR;
constexpr uint8_t kBackLeft = BL;
constexpr uint8_t kBackRight = BR;
}

namespace VlxID {
constexpr uint8_t kNone = 0;
constexpr uint8_t VLX1 = 1;
constexpr uint8_t VLX2 = 2;
constexpr uint8_t VLX3 = 3;
constexpr uint8_t VLX4 = 4;
constexpr uint8_t VLX5 = 5;
constexpr uint8_t VLX6 = 6;
constexpr uint8_t VLX7 = 7;
constexpr uint8_t VLX8 = 8;
constexpr uint8_t kCount = 8;

// Set each position to the number printed on its VLX connector.
constexpr uint8_t VLX_F1 = VLX1;
constexpr uint8_t VLX_F2 = VLX3;
constexpr uint8_t VLX_R1 = VLX8;
constexpr uint8_t VLX_R2 = kNone;
constexpr uint8_t VLX_B1 = VLX7;
constexpr uint8_t VLX_B2 = kNone;
constexpr uint8_t VLX_L1 = VLX5;
constexpr uint8_t VLX_L2 = kNone;
// Temporary: this sensor has no XSHUT and keeps 0x29 on Wire.
constexpr uint8_t kWithoutXshut = VLX_L1;

constexpr uint8_t kFront = 0;
constexpr uint8_t kRight = 1;
constexpr uint8_t kBack = 2;
constexpr uint8_t kLeft = 3;
constexpr uint8_t kSideCount = 4;
constexpr uint8_t kBySide[kSideCount][2] = {
    {VLX_F1, VLX_F2}, {VLX_R1, VLX_R2}, {VLX_B1, VLX_B2}, {VLX_L1, VLX_L2},
};

constexpr bool validMap() {
    unsigned used = 0;
    for (const auto &side : kBySide) {
        for (uint8_t id : side) {
            if (id == kNone) continue;
            if (id > kCount || (used & (1u << id))) return false;
            used |= 1u << id;
        }
    }
    return VLX_F1 != kNone || VLX_F2 != kNone;
}

constexpr uint8_t sideOf(uint8_t id) {
    if (id == kNone) return kSideCount;
    for (uint8_t side = 0; side < kSideCount; ++side) {
        for (uint8_t assigned : kBySide[side]) {
            if (assigned == id) return side;
        }
    }
    return kSideCount;
}

constexpr bool isInstalled(uint8_t id) {
    return sideOf(id) != kSideCount;
}
static_assert(validMap(), "Use unique VLX connectors and assign at least one front sensor");
}

namespace Pins {
constexpr uint8_t kMotorPwm[MotorID::kCount]  = { 6, 11, 12, 28};
constexpr uint8_t kMotorIn1[MotorID::kCount]  = { 7, 10, 25, 27};
constexpr uint8_t kMotorIn2[MotorID::kCount]  = { 8,  9, 24, 26};
constexpr uint8_t kEncoderC1[MotorID::kCount] = { 4, 20, 30, 32};
constexpr uint8_t kEncoderC2[MotorID::kCount] = { 5, 15, 31, 33};

// Positive power must move the wheel forward. If a wheel spins backwards, flip it here.
constexpr bool kMotorInverted[MotorID::kCount]   = {false, false, false, true};
// Moving forward must count positive. If a wheel counts negative, flip it here.
constexpr bool kEncoderInverted[MotorID::kCount] = {true, false, false, false};

// XSHUT pins in connector order, VLX1 through VLX8.
constexpr uint8_t kVlxXshut[VlxID::kCount] = {0, 1, 2, 23, 22, 40, 39, 38};

// Wire -> VL53L0X bus. Wire1 -> BNO bus.
constexpr uint8_t kI2cSda = 18;
constexpr uint8_t kI2cScl = 19;
constexpr uint8_t kI2c1Sda = 17;
constexpr uint8_t kI2c1Scl = 16;
}

namespace Comms {
constexpr uint32_t kUsbBaudRate = 115200;
}

namespace Geometry {
constexpr float kPi = 3.14159265f;
constexpr float kWheelDiameterCm = 4.7f;
constexpr float kTrackWidthCm = 15.0f;

constexpr float kEncoderTicksPerMotorRev = 22.0f;
constexpr float kGearRatio = 30.0f;  // Motor revolutions per wheel revolution.

constexpr float kTicksPerWheelRev = kEncoderTicksPerMotorRev * kGearRatio;
constexpr float kNominalCmPerTick = (kPi * kWheelDiameterCm) / kTicksPerWheelRev;
constexpr float kDistanceCalibrationScale = 0.4994283f * (28.5f / 30.0f);
constexpr float kCmPerTick = kNominalCmPerTick * kDistanceCalibrationScale;

constexpr float kTileSizeCm = 30.0f;
} 

namespace Control {
constexpr uint32_t kPeriodMs = 10;  // Control loop: 100 Hz
constexpr uint8_t kPwmResolutionBits = 8;
constexpr float kPwmMax = 255.0f;
constexpr float kPwmFrequencyHz = 20000.0f;  // High frequency reduces audible motor whine.

// Rescue Maze 2026 wheel PID and speed windows.
constexpr uint32_t kWheelPidPeriodMs = 20;
constexpr uint32_t kEncoderSpeedWindowMs = 40;
constexpr float kTurnPwmStepMax = 10.0f;  // Max PWM change per fresh encoder sample while turning.
constexpr float kWheelKp = 1.0f;
constexpr float kWheelKi = 0.0f;
constexpr float kWheelKd = 1.35f;
} 

namespace Motion {
constexpr float kCruiseSpeedCmS = 20.0f;
constexpr float kTurnSpeedCmS = 8.0f;
constexpr float kTurnToleranceDegrees = 0.5f;
constexpr float kBnoTurnMinPwm = 80.0f;
constexpr float kBnoTurnMaxPwm = 170.0f;
constexpr float kMinSpeedCmS = 5.0f;
constexpr float kDecelGain = 3.0f;  // Speed = gain * remaining cm, slows near the goal
constexpr float kDistanceToleranceCm = 0.5f;
constexpr uint32_t kMoveTimeoutMs = 10000;
}  // namespace Motion

namespace MazeConfig {
constexpr uint8_t kWhiteTileCost = 1;
constexpr uint8_t kBlueTileCost = 21;
constexpr uint8_t kRampTileCost = 7;
constexpr uint8_t kBlackTileCost = 100;

constexpr uint16_t kVlxPeriodMs = 40;
constexpr uint16_t kFrontWallDistanceMm = 180;
constexpr uint16_t kSideWallDistanceMm = 210;
constexpr uint16_t kBackWallDistanceMm = 180;

constexpr uint16_t wallDistanceMm(uint8_t side) {
    if (side == VlxID::kFront) return kFrontWallDistanceMm;
    if (side == VlxID::kBack) return kBackWallDistanceMm;
    return kSideWallDistanceMm;
}

constexpr uint16_t kWallReferenceMaxMm = 650; // Maximum starting distance to a reference wall.
constexpr uint16_t kFrontStopMm = 35;       // Minimum clearance in the direction of travel.
constexpr uint16_t kCenterFrontMm = 115; // Farther front VLX at the desired position; the other reads about 80 mm.
constexpr uint16_t kCenterBackMm = 88;
constexpr uint16_t kCenterToleranceMm = 5;
constexpr float kTravelSpeedCmS = 16.0f; 
constexpr bool kInvertBnoHeading = false;  // A right turn must increase heading.
constexpr bool kUseRollForRamp = false;    // Set according to the BNO mounting.
constexpr float kRampAngleDegrees = 18.0f;
constexpr bool kUseFloorColor = false;     // TCS34725 directly on Wire1 at 0x29.
}

#endif
