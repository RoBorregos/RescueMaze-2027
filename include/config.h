#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// 4 motor array in order, each with its position.

namespace MotorID {
constexpr uint8_t kFrontLeft = 0;
constexpr uint8_t kFrontRight = 1;
constexpr uint8_t kBackLeft = 2;
constexpr uint8_t kBackRight = 3;
}

// Missing:
//  -Encoder and Motors pins
//  -Wheel diameter and ticks per spin (cm_per_tick)
//  -Distance sensors ID's

#endif