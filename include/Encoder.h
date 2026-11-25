#ifndef RESCUE_ENCODER_H
#define RESCUE_ENCODER_H

#include <stdint.h>

class Motor;

namespace EncoderInterrupts {
void beginEncoders(Motor *motors);
void backRightEncoder();
void backLeftEncoder();
void frontRightEncoder();
void frontLeftEncoder();
}

#endif
