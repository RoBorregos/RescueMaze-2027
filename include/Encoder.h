#pragma once

#include "Pins_ID.h"
#include "motors.h"
#include <Arduino.h>
extern motors robot;
namespace Interrupts {
void backRightEncoder();
void backLeftEncoder();
void frontRightEncoder();
void frontLeftEncoder();
void lackOfProgress();
extern int deltaTics[4];
}; // namespace Interrupts
