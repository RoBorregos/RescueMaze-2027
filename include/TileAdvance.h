#pragma once

#include "config.h"

class TileAdvance {
public:
  enum class Reference { kEncoder, kFront, kBack };

  TileAdvance(Reference reference, float targetCm, float encoderTargetCm)
      : reference_(reference), targetCm_(targetCm), encoderTargetCm_(encoderTargetCm) {}

  Reference reference() const { return reference_; }

  bool canAdoptFront() const {
    return reference_ == Reference::kEncoder && !adoptedFront_;
  }

  bool adoptFront(float encoderCm, float wallCm) {
    if (!canAdoptFront() || encoderCm >= encoderTargetCm_) return false;
    constexpr float centerCm = MazeConfig::kCenterFrontMm / 10.0f;
    const float gap = wallCm > centerCm ? wallCm - centerCm : 0.0f;
    const float proposedTarget = encoderCm + gap;
    if (proposedTarget > encoderTargetCm_) return false;
    reference_ = Reference::kFront;
    adoptedFront_ = true;
    targetCm_ = centerCm;
    encoderTargetCm_ = proposedTarget;
    return true;
  }

  float remainingCm(float encoderCm, bool validWall, float wallCm) {
    if (reference_ != Reference::kEncoder && !validWall) {
      reference_ = Reference::kEncoder;
    }
    const float encoderRemaining = encoderTargetCm_ - encoderCm;
    if (reference_ == Reference::kEncoder) return encoderRemaining;
    const float wallRemaining = reference_ == Reference::kFront ?
        wallCm - targetCm_ : targetCm_ - wallCm;
    if (!adoptedFront_) return wallRemaining;
    return wallRemaining < encoderRemaining ? wallRemaining : encoderRemaining;
  }

  bool done(float remaining) const {
    if (reference_ == Reference::kEncoder) return remaining <= 0.0f;
    return remaining <= MazeConfig::kCenterToleranceMm / 10.0f;
  }

private:
  Reference reference_;
  float targetCm_;
  float encoderTargetCm_;
  bool adoptedFront_ = false;
};

class FrontWallCandidate {
public:
  bool observe(uint8_t slot, float wallCm, float encoderCm, uint32_t sampleTimeMs) {
    if (pending_ && slot == slot_ && sampleTimeMs == sampleTimeMs_) return false;
    const float predictionError = wallCm - wallCm_ + encoderCm - encoderCm_;
    const float tolerance = MazeConfig::kWallReferenceJumpMm / 10.0f;
    const bool confirmed = pending_ && slot == slot_ &&
                           predictionError >= -tolerance && predictionError <= tolerance;
    pending_ = true;
    slot_ = slot;
    wallCm_ = wallCm;
    sampleTimeMs_ = sampleTimeMs;
    encoderCm_ = encoderCm;
    return confirmed;
  }

  void reset() { pending_ = false; }

private:
  bool pending_ = false;
  uint8_t slot_ = 0;
  uint32_t sampleTimeMs_ = 0;
  float wallCm_ = 0.0f;
  float encoderCm_ = 0.0f;
};
