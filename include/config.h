#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// 4 motor array in order, each with its position.

//Everything marked TODO must be replaced

namespace MotorID {
constexpr uint8_t kFrontLeft = 0;
constexpr uint8_t kFrontRight = 1;
constexpr uint8_t kBackLeft = 2;
constexpr uint8_t kBackRight = 3;
constexpr uint8_t kCount = 4;
}

//34, 35, 36 y 39 --> Input pins
//TODO -> Assumes a driver with a 2 direction pins
namespace Pins{
    
    //TODO Replace with real pins
    constexpr uint8_t kMotorIn1[MotorID::kCount] = {16, 18, 25, 32};
    constexpr uint8_t kMotorIn2[MotorID::kCount] = {17, 19, 26, 33};
    constexpr uint8_t kMotorPwm[MotorID::kCount] = {4, 13, 27, 14};

    //TODO single-channel encoders 
    constexpr uint8_t kEncoderA[MotorID::kCount] = {34, 35, 36, 39};

    //I2C Slave bus
    constexpr uint8_t kI2cSda = 21;
    constexpr uint8_t kI2cScl = 22;
}

namespace Comms{
    constexpr uint8_t kI2cSlaveAddres = 0x12; //TODO Replace with real address
}

namespace Geometry {
    constexpr float kPi = 3.14159265f;
    constexpr float kWheelDiameter = 7.0f; //TODO Replace with real diameter in cm
    constexpr float kTrackWidthCm = 15.0f; //TODO Replace with real track width in cm

    //Encoder readings per motor revolution
    constexpr float kEncoderCountsPerMotorRev = 11.0f; //TODO Replace with real value

    //Motor revolutions needed for a full wheel revolution
    constexpr float kGearRatio = 30.0f; //TODO Replace with real value

    //Ticks readings per wheel revolution
    constexpr float kTicksPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;

    //Cm traveled per encoder tick
    constexpr float kCmPerTick = (kPi * kWheelDiameter) / kTicksPerWheelRev;

    constexpr float kTileSizeCm = 28.0f;  // Rescue Maze 2027 rules
}

namespace Control {
    
    //Calculate PID and fix errors every 10ms, 10,000 times per second, 100Hz
    constexpr uint32_t kPeriodMs = 10; // fixed control loop period
    constexpr float kPwmMax = 255.0f; // 8-bit PWM

    //TODO -> Replace with real values
    constexpr float kWheelKp = 2.0f;
    constexpr float kWheelKi = 10.f;
    constexpr float kWheelKd = 0.0f;
}

namespace Motion {
    //TODO -> tune
    constexpr float kCruiseSpeedCmS = 20.0f;
    constexpr float kTurnSpeedCmS = 15.0f;
}

// Missing:
//  -Distance sensors ID's

#endif