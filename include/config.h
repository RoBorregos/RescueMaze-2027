#pragma once
#include <stdint.h>

namespace Comms {
constexpr uint32_t kUsbBaudRate = 115200;
constexpr uint32_t kI2cClockHz = 100000;
constexpr uint8_t kOledAddress = 0x3C;
constexpr uint32_t kButtonDebounceMs = 300;
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
// Apply the existing distance correction to the counts from channel A rising edges.
// Verify the scale against the measured distance of a 30 cm floor test.
constexpr float kRisingTickDistanceScale = 0.94891377f;
constexpr float kCmPerRisingTick =
    (kPi * kWheelDiameterCm / kTicksPerWheelRev) * kRisingTickDistanceScale;
constexpr float kTileSizeCm = 30.0f;
}

namespace EncoderConfig {
// Start the loss timer only while a wheel is receiving meaningful power.
constexpr uint16_t kWatchPwm = 50;
constexpr uint32_t kNoPulsesMs = 300;
} // namespace EncoderConfig

namespace Control {
constexpr float kPwmMax = 255.0f;
constexpr float kPwmFrequencyHz = 20000.0f;  // High frequency reduces audible motor whine.

// Wheel PID and speed sampling.
constexpr uint32_t kWheelPidPeriodMs = 20;
constexpr uint32_t kEncoderSpeedWindowMs = 40;
constexpr float toWheelPidTicks(float speedCmS) {
    return speedCmS * (kEncoderSpeedWindowMs / 1000.0f) / Geometry::kCmPerRisingTick;
}
constexpr float kWheelKp = 1.0f;
constexpr float kWheelKi = 0.0f;
constexpr float kWheelKd = 1.35f;
constexpr float kHeadingKp = 0.5f;
constexpr float kHeadingKi = 0.1f;
constexpr float kHeadingKd = 0.01f;
constexpr uint32_t kHeadingPidPeriodMs = 1;
constexpr float kHeadingCorrectionMax = 17.0f;
constexpr float kRampUpCorrectionMax = 15.0f;
constexpr float kRampDownCorrectionMax = 6.0f;
constexpr int kMinForwardPwm = 70;
constexpr int kMaxForwardPwm = 180;
constexpr bool kDebugPid = false;
}

namespace Motion {
constexpr float kTurnSpeedCmS = 8.0f;
constexpr float kTurnToleranceDegrees = 0.5f;
constexpr float kEncoderTurnToleranceDegrees = 2.0f;
// Initial estimates based on BNO measurements on the floor.
constexpr float kEncoderRightTurnScale = 0.65f;
constexpr float kEncoderLeftTurnScale = 0.85f;
constexpr float scaleEncoderTurn(float degrees) {
    return degrees * (degrees < 0.0f ? kEncoderLeftTurnScale : kEncoderRightTurnScale);
}
constexpr int kNoEncoderMinPwm = 55;
constexpr int kNoEncoderMaxPwm = 100;
constexpr uint32_t kTurnTimeoutMs = 5000;
constexpr uint32_t kHalfTurnTimeoutMs = 10000;
constexpr float kHalfTurnMinDegrees = 175.0f;
constexpr float kTurnSpeedGain = 0.24f;
constexpr float kMinTurnSpeedCmS = 2.5f;
constexpr float kObstacleTurnDegrees = 25.0f;
constexpr uint8_t kObstacleRetreatCm = Geometry::kTileSizeCm / 5;
constexpr uint8_t kObstaclePassCm = 3 * Geometry::kTileSizeCm / 10;
constexpr float kSlopeAngleDegrees = 7.0f;
constexpr float kStartOnSlopeAngleDegrees = 15.0f;
constexpr float kCheckpointAngleToleranceDegrees = 10.0f;
constexpr float kMinSpeedCmS = 5.0f;
constexpr float kDecelGain = 3.0f;  // Speed = gain * remaining cm, slows near the goal
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
constexpr float kLateralClearanceCm = 6.0f;
constexpr float kRampWallDistanceCm = 15.0f;
constexpr float kRampFrontDifferenceCm = 2.0f;
constexpr uint32_t kFloorColorSampleMs = 50;
constexpr uint32_t kBlackTileRetreatTimeoutMs = 3000;
constexpr uint32_t kBlueTileWaitMs = 5500;
}

namespace BnoConfig {
constexpr uint8_t kAddresses[] = {0x4A, 0x4B};
constexpr uint32_t kReportFreshnessMs = 250;
constexpr uint32_t kSetupTimeoutMs = 1000;
}

namespace VlxConfig {
constexpr uint16_t kRangePeriodMs = 100;
constexpr uint32_t kRangeFreshnessMs = 2 * kRangePeriodMs;
constexpr float kOutOfRangeCm = 819.0f;
}

namespace KitConfig {
constexpr uint16_t kLeftReleaseDegrees = 10;
constexpr uint16_t kCenterDegrees = 90;
constexpr uint16_t kRightReleaseDegrees = 170;
constexpr uint16_t kWiggleDegrees = 6;
constexpr uint32_t kServoSettleMs = 200;
constexpr uint32_t kWiggleStepMs = 40;
constexpr uint32_t kVictimSequenceMs = 5100;
constexpr uint32_t kVictimBlinkMs = 500;
}
