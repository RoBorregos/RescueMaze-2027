#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// Test robot. Pins taken from the test schematic (EsquematicoPrueba).
// Teensy 4.1 + 2x TB6612FNG + 2-channel encoders.

namespace MotorID {
constexpr uint8_t kFrontLeft = 0;   // M1
constexpr uint8_t kFrontRight = 1;  // M2
constexpr uint8_t kBackLeft = 2;    // M3
constexpr uint8_t kBackRight = 3;   // M4
constexpr uint8_t kCount = 4;
}

namespace Pins {                                //FL, FR, BL, BR               
constexpr uint8_t kMotorPwm[MotorID::kCount]  = { 2, 3, 4, 5};
constexpr uint8_t kMotorIn1[MotorID::kCount]  = { 30, 32, 34, 36};
constexpr uint8_t kMotorIn2[MotorID::kCount]  = { 31, 33, 35, 37};
constexpr uint8_t kEncoderC1[MotorID::kCount] = { 9, 11, 14, 20};
constexpr uint8_t kEncoderC2[MotorID::kCount] = { 10, 12, 15, 21};  

// Positive power must move the wheel forward. If a wheel spins backwards, flip it here.
constexpr bool kMotorInverted[MotorID::kCount]   = {false, false, false, true};
// Moving forward must count positive. If a wheel counts negative, flip it here.
constexpr bool kEncoderInverted[MotorID::kCount] = {false, false, false, true};

// Limit switches LS_L / LS_R are not wired to the Teensy in the schematic yet.

// VL53L0X XSHUT lines: FB, BF, LR, RL, M (not used by the control code yet)
constexpr uint8_t kVlxXshut[5] = {22, 26, 23, 27, 38};

// I2C buses
// Wire  (SDA 18 / SCL 19): BNO085 + TCS34725 color sensor
// Wire1 (SDA 17 / SCL 16): 4x VL53L0X
// Wire2 (SDA 25 / SCL 24): 5x VL53L0X
// Cameras: right OpenMV on Serial2 (pins 7/8), left OpenMV on Serial7 (pins 28/29)

}

namespace Comms {
constexpr uint32_t kUsbBaudRate = 115200;  // Test output over USB
}

namespace Geometry {
constexpr float kPi = 3.14159265f;
constexpr float kWheelDiameterCm = 6.2f;
constexpr float kTrackWidthCm = 15.0f;

//Pololu 100:1 HPCB
constexpr float kEncoderTicksPerMotorRev = 12.0f;

// motor.cpp only counts the edges of channel C1: 2 of the 4 edges.
constexpr float kCountedEdgeFraction = 0.5f;

constexpr float kGearRatio = 100.37f;

constexpr float kTicksPerWheelRev = kEncoderTicksPerMotorRev * kCountedEdgeFraction * kGearRatio; //~602 
constexpr float kCmPerTick = (kPi * kWheelDiameterCm) / kTicksPerWheelRev;

constexpr float kTileSizeCm = 28.0f;
} 

namespace Control {
constexpr uint32_t kPeriodMs = 10;  // Control loop: 100 Hz
constexpr uint8_t kPwmResolutionBits = 8;
constexpr float kPwmMax = 255.0f;
constexpr float kPwmFrequencyHz = 20000.0f;  // Above hearing range, motors do not buzz

constexpr float kWheelKp = 2.0f;
constexpr float kWheelKi = 10.0f;
constexpr float kWheelKd = 0.0f;
} 

namespace ImuConfig{
    constexpr uint8_t kAddress = 0x4A; //I2C address of the BNO085 IMU
    constexpr uint32_t kReportIntervalUs = 10000; //New heading every 10ms (100Hz)

    // Turning the robot clockwise must make the heading grow. If it decreases, the board is mounted upside down, so set this to true.
    constexpr bool kHeadingInverted = false;
}

namespace Motion {
constexpr float kCruiseSpeedCmS = 20.0f;
constexpr float kTurnSpeedCmS = 15.0f;
constexpr float kMinSpeedCmS = 5.0f;
constexpr float kDecelGain = 3.0f;  // Speed = gain * remaining cm, slows near the goal
constexpr float kDistanceToleranceCm = 0.5f;
constexpr uint32_t kMoveTimeoutMs = 5000;
constexpr float kTurnKp = 0.5f;
constexpr float kTurnKd = 0.02f;
constexpr float kMinTurnSpeedCmS = 4.0f; //Below this wheels wont move
constexpr float kTurnToleranceDeg = 1.5f;
constexpr uint8_t kTurnSettleCycles = 5; //Number of cycles the turn must be within tolerance before it is considered done

//Extra speed per degree of heading error
constexpr float kHeadingKp = 0.8f;
constexpr float kHeadingKd = 0.02f;
constexpr float kMaxHeadingCorrectionCmS = 6.0f; //Extra speed added to the wheels to correct heading error
}

#endif
