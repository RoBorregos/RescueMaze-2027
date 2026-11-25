#pragma once
#include <stdint.h>

namespace Comms {
constexpr uint32_t kUsbBaudRate = 115200;
}

namespace Startup {
// Settle after sensor and motor setup, before running DFS.
constexpr uint32_t kSettleMs = 500;
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

// The 2027 firmware counts both edges of channel A; this port counts only rising edges.
constexpr float kCmPerRisingTick = 2.0f * kCmPerTick;
constexpr float kTileSizeCm = 30.0f;
}

namespace EncoderConfig {
// Start the loss timer only while a wheel is receiving meaningful power.
constexpr uint16_t kWatchPwm = 50;
constexpr uint32_t kNoPulsesMs = 300;
} // namespace EncoderConfig

namespace Control {
constexpr uint32_t kPeriodMs = 10;  // Control loop: 100 Hz
constexpr uint8_t kPwmResolutionBits = 8;
constexpr float kPwmMax = 255.0f;
constexpr float kPwmFrequencyHz = 20000.0f;  // High frequency reduces audible motor whine.

// Rescue Maze 2026 wheel PID and speed windows.
constexpr uint32_t kWheelPidPeriodMs = 20;
constexpr uint32_t kEncoderSpeedWindowMs = 40;
constexpr float toWheelPidTicks(float speedCmS) {
    return speedCmS * (kEncoderSpeedWindowMs / 1000.0f) / Geometry::kCmPerRisingTick;
}
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
}

namespace MazeConfig {
// Wall classification thresholds, relative to the robot's heading.
constexpr uint16_t kFrontWallDistanceMm = 180;
constexpr uint16_t kSideWallDistanceMm = 210;
constexpr uint16_t kBackWallDistanceMm = 180;

// Centering and usable wall reference distances, not wall classification.
constexpr uint16_t kWallReferenceMaxMm = 650;
constexpr uint32_t kFrontWallProbeMs = 100;
constexpr uint16_t kWallReferenceJumpMm = 80; // Allowed difference from encoder-predicted range.
constexpr uint16_t kFrontStopMm = 35;
constexpr uint16_t kCenterFrontMm = 115;
constexpr uint16_t kCenterBackMm = 88;
constexpr uint16_t kCenterToleranceMm = 5;
constexpr float kTravelSpeedCmS = 16.0f;
constexpr float kRampAngleDegrees = 18.0f;
constexpr bool kUseFloorColor = true;
}
