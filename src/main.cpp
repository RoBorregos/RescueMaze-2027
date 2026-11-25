#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "BNO.h"
#include "VLX.h"
#include "config.h"
#include "maze.h"
#include "motors.h"
#include "movements.h"

namespace {
constexpr uint32_t kStartupDelayMs = 3000;
constexpr uint32_t kInitialProgressWindowMs = 750;
constexpr uint32_t kProgressWindowMs = 350;
constexpr uint32_t kEncoderRetryPauseMs = 150;
constexpr uint32_t kObstaclePollPeriodMs = 25;
constexpr uint32_t kWallReferenceLossMs = 300;
constexpr uint32_t kFrontReadingFreshMs = 2 * MazeConfig::kVlxPeriodMs;
constexpr uint32_t kFrontPairWaitMs = MazeConfig::kVlxPeriodMs + 20;
constexpr float kMinWallProgressCm = 0.3f;
constexpr float kRetryAdvanceProgressCm = 1.0f;
constexpr float kRetryTurnProgressDegrees = 2.0f;
constexpr uint32_t kHalfTurnTimeoutMs = 2 * Motion::kMoveTimeoutMs;
constexpr uint8_t kAllSides = 0x0F;
constexpr uint8_t kWallReferenceSides = (1u << VlxID::kFront) | (1u << VlxID::kBack);
constexpr float kMinProgressCm = 0.2f;
constexpr float kMaxHeadingError = 22.0f;
constexpr float kWrongTurnAllowance = 15.0f;

Chassis chassis;
Movements movement;
Orientation bno;
OrientationQuaternion origin{};
bool haveOrigin = false;
VLX ranges[VlxID::kCount];
RescueMaze maze;
uint8_t facing = 0;
bool exploring = false;
bool stopped = false;

bool halt(const char *reason) {
    stopped = true;
    movement.cancel();
    Serial.println(reason);
    return false;
}

bool stopRequested(bool *resume = nullptr) {
    if (stopped) return true;
    while (Serial.available()) {
        const int command = Serial.read();
        if (command == 'x') {
            halt("Paro: comando x por USB.");
            return true;
        }
        if (command == 'r' && resume != nullptr) *resume = true;
    }
    return false;
}

bool initRanges() {
    if (!VLX::prepareBus()) return halt("Paro: no se pudo preparar el bus de los VLX.");
    for (uint8_t i = 0; i < VlxID::kCount; ++i) {
        if (i + 1 == VlxID::kWithoutXshut || !VlxID::isInstalled(i + 1)) continue;
        if (stopRequested()) return false;
        ranges[i].setSensorIndex(i);
        if (ranges[i].begin()) Serial.printf("VLX%u activo.\n", unsigned(i + 1));
    }
    // Restore 0x29 only after all sensors with XSHUT have been addressed.
    if (VlxID::isInstalled(VlxID::kWithoutXshut)) {
        if (stopRequested()) return false;
        const uint8_t i = VlxID::kWithoutXshut - 1;
        ranges[i].setSensorIndex(i);
        ranges[i].begin();
    }
    return true;
}

bool readHeading(float &heading) {
    OrientationQuaternion sample{};
    return bno.readQuaternion(sample) && clockwiseHeadingOffset(origin, sample, heading);
}

bool ensureOrigin() {
    if (haveOrigin) return true;
    if (!bno.readQuaternion(origin)) return halt("Paro: sin rumbo del BNO para iniciar el movimiento.");
    haveOrigin = true;
    return true;
}

struct RangeObservation {
    VlxReading reading = VlxReading::kUnknown;
    uint16_t millimeters = 0;
};

bool startRangeCapture(uint8_t sides, uint16_t &pending) {
    pending = 0;
    for (uint8_t side = 0; side < VlxID::kSideCount; ++side) {
        if (!(sides & (1u << side))) continue;
        for (uint8_t id : VlxID::kBySide[side]) {
            if (id == VlxID::kNone || !ranges[id - 1].initialized) continue;
            if (stopRequested()) return false;
            ranges[id - 1].startNavigationRead();
            pending |= 1u << (id - 1);
        }
    }
    return true;
}

void pollRangeCapture(uint16_t &pending, RangeObservation (&observations)[VlxID::kCount]) {
    for (uint8_t i = 0; i < VlxID::kCount; ++i) {
        const uint16_t bit = 1u << i;
        if (!(pending & bit)) continue;
        auto &observation = observations[i];
        if (ranges[i].pollNavigationRead(observation.reading, observation.millimeters)) pending &= ~bit;
    }
}

bool finishRangeCapture(uint16_t &pending, RangeObservation (&observations)[VlxID::kCount]) {
    while (pending != 0) {
        if (stopRequested()) return false;
        pollRangeCapture(pending, observations);
        if (pending != 0) delay(1);
    }
    return true;
}

bool captureRanges(uint8_t sides, RangeObservation (&observations)[VlxID::kCount]) {
    uint16_t pending = 0;
    return startRangeCapture(sides, pending) && finishRangeCapture(pending, observations);
}

struct TurnRangeCapture {
    RangeObservation observations[VlxID::kCount]{};
    uint16_t pending = 0;
    bool started = false;
};

bool captureWhileSettling(TurnRangeCapture &capture, float turnError) {
    if (!movement.isTurnSettling() || fabsf(turnError) > Motion::kTurnToleranceDegrees) {
        if (movement.getStatus() != MoveStatus::kDone) capture.started = false;
        return true;
    }
    if (!capture.started) {
        capture = {};
        if (!startRangeCapture(kWallReferenceSides, capture.pending)) return false;
        capture.started = true;
    }
    pollRangeCapture(capture.pending, capture.observations);
    return true;
}

VlxReading readSide(uint8_t side, const RangeObservation (&observations)[VlxID::kCount],
                    uint16_t &nearest, bool report = true) {
    nearest = UINT16_MAX;
    bool noReturn = false;
    for (uint8_t id : VlxID::kBySide[side]) {
        if (id == VlxID::kNone || !ranges[id - 1].initialized) continue;
        const auto &observation = observations[id - 1];
        if (observation.reading == VlxReading::kDistance) {
            if (report) Serial.printf("VLX%u: %u mm.\n", unsigned(id), unsigned(observation.millimeters));
            if (observation.millimeters < nearest) nearest = observation.millimeters;
        } else if (observation.reading == VlxReading::kNoReturn) {
            if (report) Serial.printf("VLX%u: sin retorno repetido.\n", unsigned(id));
            noReturn = true;
        } else if (report) {
            Serial.printf("VLX%u: sin lectura valida.\n", unsigned(id));
        }
    }
    if (nearest != UINT16_MAX) return VlxReading::kDistance;
    return noReturn ? VlxReading::kNoReturn : VlxReading::kUnknown;
}

enum class WallState { kUnknown, kOpen, kWall };

WallState wallOnSide(uint8_t side, const RangeObservation (&observations)[VlxID::kCount], bool report = false) {
    const uint16_t threshold = MazeConfig::wallDistanceMm(side);
    uint16_t distance = 0;
    // The closest valid reading takes priority over a distant reading or no echo.
    const VlxReading reading = readSide(side, observations, distance, report);
    if (reading == VlxReading::kUnknown) return WallState::kUnknown;
    return reading == VlxReading::kDistance && distance < threshold ? WallState::kWall : WallState::kOpen;
}

struct WallReference {
    uint8_t sensorId = VlxID::kNone;
    bool behind = false;
    uint16_t initialMm = 0;
    uint16_t latestMm = 0;
    uint32_t lastReadingMs = 0;
    float startingTravelCm = 0.0f;
    float traveledCm = 0.0f;
};

struct FrontReading {
    uint8_t sensorId = VlxID::kNone;
    uint16_t millimeters = 0;
    uint32_t sampledAt = 0;
    float encoderTravelCm = 0.0f;
};

struct FrontReadings {
    FrontReading samples[2]{};
    uint32_t singleSince = 0;
    bool waitingForPair = false;
};

float encoderTravelCm() {
    return (chassis.getLeftDistance() + chassis.getRightDistance()) / 2.0f;
}

WallReference wallReferenceOnSide(uint8_t side, const RangeObservation (&observations)[VlxID::kCount]) {
    WallReference reference{};
    reference.behind = side == VlxID::kBack;
    const bool takeFarther = side == VlxID::kFront;
    for (uint8_t id : VlxID::kBySide[side]) {
        if (id == VlxID::kNone) continue;
        const auto &observation = observations[id - 1];
        if (observation.reading != VlxReading::kDistance) continue;
        const bool better = takeFarther ? observation.millimeters > reference.initialMm :
                                          observation.millimeters < reference.initialMm;
        if (reference.sensorId == VlxID::kNone || better) {
            reference.sensorId = id;
            reference.initialMm = observation.millimeters;
            reference.latestMm = observation.millimeters;
        }
    }
    return reference.initialMm <= MazeConfig::kWallReferenceMaxMm ? reference : WallReference{};
}

void updateWallReference(WallReference &reference, uint16_t millimeters, uint32_t now, bool reversing) {
    const int32_t differenceMm = int32_t(millimeters) - reference.initialMm;
    const int32_t traveledMm = reference.behind ? differenceMm : -differenceMm;
    const float traveledCm = reference.startingTravelCm + traveledMm / 10.0f;
    const float progress = traveledCm - reference.traveledCm;
    if ((reversing ? -progress : progress) < kMinWallProgressCm) return;
    reference.traveledCm = traveledCm;
    reference.latestMm = millimeters;
    reference.lastReadingMs = now;
}

bool advanceObstacle(WallReference &reference, uint32_t now, bool reversing, float encoderTravel,
                     FrontReadings &front) {
    const uint8_t leadingSide = reversing ? VlxID::kBack : VlxID::kFront;
    const auto &sensors = VlxID::kBySide[leadingSide];
    for (uint8_t slot = 0; slot < 2; ++slot) {
        const uint8_t id = sensors[slot];
        if (id == VlxID::kNone) continue;
        uint16_t millimeters = 0;
        if (!ranges[id - 1].readReadyMillimeters(millimeters)) continue;
        if (millimeters < MazeConfig::kFrontStopMm) return true;
        if (!reversing) front.samples[slot] = {id, millimeters, now, encoderTravel};
        if (id == reference.sensorId) updateWallReference(reference, millimeters, now, reversing);
    }
    if (reference.behind != reversing && reference.sensorId != VlxID::kNone) {
        uint16_t millimeters = 0;
        if (ranges[reference.sensorId - 1].readReadyMillimeters(millimeters)) {
            updateWallReference(reference, millimeters, now, reversing);
        }
    }
    return false;
}

FrontReading farthestFrontReading(FrontReadings &front, uint32_t now, bool expectPair) {
    FrontReading farthest{};
    uint8_t fresh = 0;
    for (const auto &sample : front.samples) {
        if (sample.sensorId == VlxID::kNone || now - sample.sampledAt >= kFrontReadingFreshMs) continue;
        ++fresh;
        if (sample.millimeters > farthest.millimeters) farthest = sample;
    }
    if (expectPair && fresh == 1) {
        if (!front.waitingForPair) {
            front.waitingForPair = true;
            front.singleSince = now;
        }
        if (now - front.singleSince < kFrontPairWaitMs) return {};
    } else {
        front.waitingForPair = false;
    }
    return farthest;
}

void applyFrontAdvanceLimit(WallReference &reference, FrontReading front, uint32_t now) {
    if (front.sensorId == VlxID::kNone || front.millimeters > MazeConfig::kWallReferenceMaxMm) return;
    if (front.sensorId != reference.sensorId) {
        if (reference.sensorId != VlxID::kNone && !reference.behind &&
            now - reference.lastReadingMs < kFrontReadingFreshMs && front.millimeters <= reference.latestMm) return;
        reference = {};
        reference.sensorId = front.sensorId;
        reference.initialMm = front.millimeters;
        reference.latestMm = front.millimeters;
        reference.lastReadingMs = front.sampledAt;
        reference.startingTravelCm = front.encoderTravelCm;
        reference.traveledCm = front.encoderTravelCm;
        Serial.printf("Referencia frontal adquirida durante el avance: VLX%u a %u mm.\n",
                      unsigned(front.sensorId), unsigned(front.millimeters));
    }

    const float remainingMm = int32_t(front.millimeters) - MazeConfig::kCenterFrontMm;
    if (remainingMm <= 0.0f) {
        const float measuredTravel = reference.startingTravelCm + (int32_t(reference.initialMm) - front.millimeters) / 10.0f;
        reference.traveledCm = max(reference.traveledCm, measuredTravel);
        reference.lastReadingMs = front.sampledAt;
    }
    const float limitCm = reference.startingTravelCm +
        (int32_t(reference.initialMm) - MazeConfig::kCenterFrontMm) / 10.0f;
    if (movement.setForwardAdvanceLimit(limitCm)) {
        Serial.printf("Objetivo ajustado: %.1f cm para dejar %u mm al frente.\n",
                      constrain(limitCm, 0.0f, Geometry::kTileSizeCm), unsigned(MazeConfig::kCenterFrontMm));
    }
}

float advanceTravelCm(WallReference &reference, uint32_t now, float encoderTravel) {
    if (reference.sensorId == VlxID::kNone) return encoderTravel;
    if (now - reference.lastReadingMs >= kWallReferenceLossMs) {
        reference.sensorId = VlxID::kNone;
        Serial.printf("Pared fuera de lectura. Encoders desde el inicio: %.1f cm.\n", encoderTravel);
        return encoderTravel;
    }
    return reference.traveledCm;
}

enum class MazeMotion { kAdvance, kTurn, kCentering };

bool finishCenteringEarly(const char *reason) {
    movement.cancel();
    Serial.println(reason);
    return true;
}

bool waitForMotionResume() {
    Serial.println("Pausa. Envia r para reanudar el mismo movimiento o x para parar.");
    bool resume = false;
    while (!resume) {
        if (stopRequested(&resume)) return false;
        if (!resume) delay(10);
    }
    return true;
}

bool runMotion(MazeMotion kind, float heading, float targetTurn, WallReference reference = {},
               bool reversing = false, float *completedTravel = nullptr, TurnRangeCapture *turnRanges = nullptr) {
    const bool turning = kind == MazeMotion::kTurn;
    const bool centering = kind == MazeMotion::kCentering;
    if (!turning && !chassis.hasEncoderOnEachSide()) {
        return halt("Paro: para avanzar se necesita al menos un encoder por lado.");
    }
    uint32_t lastControl = millis();
    reference.lastReadingMs = lastControl;
    if (kind == MazeMotion::kAdvance && !reference.behind) {
        applyFrontAdvanceLimit(reference, {reference.sensorId, reference.initialMm, lastControl, 0.0f}, lastControl);
    }
    FrontReadings front{};
    bool expectFrontPair = true;
    for (uint8_t id : VlxID::kBySide[VlxID::kFront]) {
        if (id == VlxID::kNone || !ranges[id - 1].initialized) expectFrontPair = false;
    }
    uint32_t lastProgress = lastControl;
    uint32_t lastObstaclePoll = lastControl;
    uint32_t progressWindow = kInitialProgressWindowMs;
    float previousDistance[MotorID::kCount]{};
    uint8_t encodersToRecheck = centering ? 0 : chassis.ignoredEncoderMask();
    float turned = 0.0f;
    float traveled = 0.0f;
    float progressAtRetry = 0.0f;
    bool automaticRetryUsed = false;
    bool encoderRetryUsed = false;
    const auto resumeMotion = [&]() {
        if (!movement.resume()) return halt("Paro: no se pudo reanudar el movimiento.");
        lastControl = millis();
        lastProgress = lastControl;
        lastObstaclePoll = lastControl - kObstaclePollPeriodMs;
        progressWindow = kInitialProgressWindowMs;
        for (uint8_t i = 0; i < MotorID::kCount; ++i) {
            previousDistance[i] = chassis.getWheelTelemetry(i).distance_cm;
        }
        return true;
    };

    while (movement.isBusy()) {
        if (stopRequested()) return false;
        const uint32_t now = millis();
        const uint32_t elapsed = now - lastControl;
        if (elapsed < Control::kPeriodMs) {
            delay(1);
            continue;
        }
        lastControl = now;

        const bool settling = turning && movement.isTurnSettling();
        if (settling || now - lastProgress >= progressWindow) {
            uint8_t stalledEncoders = 0;
            for (uint8_t i = 0; i < MotorID::kCount; ++i) {
                const uint8_t bit = 1u << i;
                const WheelTelemetry wheel = chassis.getWheelTelemetry(i);
                const float progress = wheel.distance_cm - previousDistance[i];
                previousDistance[i] = wheel.distance_cm;
                if (chassis.isEncoderIgnored(i)) {
                    const float poweredProgress = wheel.pwm > 0 ? progress : -progress;
                    if (!settling && (encodersToRecheck & bit) && wheel.pwm != 0 && poweredProgress >= kMinProgressCm &&
                        chassis.restoreEncoder(i)) {
                        encodersToRecheck &= ~bit;
                        Serial.printf("M%u vuelve a medir. Encoders disponibles: %u.\n",
                                      unsigned(i + 1), unsigned(chassis.activeEncoderCount()));
                    }
                    continue;
                }
                const bool reverseWheel = turning ? wheel.pwm < 0 : reversing;
                const float directedProgress = reverseWheel ? -progress : progress;
                if (!settling && directedProgress < kMinProgressCm) {
                    Serial.printf("M%u: %+.2f cm, PWM %d.\n", unsigned(i + 1), progress, wheel.pwm);
                    if (centering) return finishCenteringEarly("Centrado parcial: poco avance en una rueda.");
                    stalledEncoders |= bit;
                }
            }
            if (stalledEncoders != 0) {
                if (!turning && !chassis.hasEncoderOnEachSide(stalledEncoders)) {
                    movement.pause();
                    if (!encoderRetryUsed) {
                        encoderRetryUsed = true;
                        Serial.println("Un lado sin encoder valido. Frenando y reintentando desde aqui.");
                        const uint32_t pauseStarted = millis();
                        while (millis() - pauseStarted < kEncoderRetryPauseMs) {
                            if (stopRequested()) return false;
                            delay(10);
                        }
                    } else {
                        Serial.println("Un lado sigue sin encoder valido.");
                        if (!waitForMotionResume()) return false;
                        encoderRetryUsed = false;
                    }
                    encodersToRecheck = chassis.ignoredEncoderMask();
                    if (!resumeMotion()) return false;
                    continue;
                }
                for (uint8_t i = 0; i < MotorID::kCount; ++i) {
                    if (!(stalledEncoders & (1u << i))) continue;
                    chassis.ignoreEncoder(i);
                    Serial.printf("Aviso: M%u excluido de la medicion.\n", unsigned(i + 1));
                }
                Serial.printf("Encoders disponibles: %u.\n", unsigned(chassis.activeEncoderCount()));
            }
            lastProgress = now;
            progressWindow = settling ? kInitialProgressWindowMs : kProgressWindowMs;
        }

        float measured = 0.0f;
        const bool haveHeading = readHeading(measured);
        float error = 0.0f;
        const float encoderTravel = turning ? 0.0f : encoderTravelCm();
        if (turning) {
            if (!haveHeading) return halt("Paro: falta lectura del BNO para continuar el giro.");
            // Accumulate BNO heading changes across the -180/+180 boundary.
            turned += angleError(measured, heading);
            heading = measured;
            error = targetTurn - turned;
            if (fabsf(error) > fabsf(targetTurn) + kWrongTurnAllowance) {
                return halt("Paro: sentido de giro inesperado. Revisa el BNO y el mapa de motores.");
            }
        } else {
            if (!haveHeading) return halt("Paro: lectura del BNO perdida.");
            error = angleError(facing * 90.0f, measured);
            if (fabsf(error) > kMaxHeadingError) return halt("Paro: desvio de rumbo durante el avance.");
            if (now - lastObstaclePoll >= kObstaclePollPeriodMs) {
                lastObstaclePoll = now;
                if (advanceObstacle(reference, now, reversing, encoderTravel, front)) {
                    if (centering) return finishCenteringEarly("Centrado parcial: se alcanzo la distancia minima.");
                    return halt(reversing ? "Paro: obstaculo cerca del VLX trasero." :
                                "Paro: obstaculo cerca del VLX frontal.");
                }
            }
        }

        if (kind == MazeMotion::kAdvance) {
            applyFrontAdvanceLimit(reference, farthestFrontReading(front, now, expectFrontPair), now);
        }
        if (!turning) {
            traveled = advanceTravelCm(reference, now, encoderTravel);
            if (centering) traveled = reversing ? min(traveled, encoderTravel) : max(traveled, encoderTravel);
            if (!centering && encoderTravel >= Geometry::kTileSizeCm) traveled = max(traveled, encoderTravel);
        }
        if (turning) movement.updateWithHeading(error);
        else movement.updateAdvanceWithHeading(error, traveled);
        if (turnRanges != nullptr && !captureWhileSettling(*turnRanges, error)) return false;
        if (!centering && movement.getStatus() == MoveStatus::kTimeout) {
            const float progress = turning ? turned : traveled;
            const float minimum = turning ? kRetryTurnProgressDegrees : kRetryAdvanceProgressCm;
            if (turning) {
                Serial.printf("Limite de giro por BNO: %.1f de %.1f grados.\n", turned, targetTurn);
            } else {
                Serial.printf("Limite de avance tras %.1f cm (encoders %.1f cm).\n", traveled, encoderTravel);
            }
            if (!automaticRetryUsed && fabsf(progress - progressAtRetry) >= minimum) {
                automaticRetryUsed = true;
                Serial.println("Reintentando solo lo que falta del movimiento.");
            } else {
                if (!waitForMotionResume()) return false;
                automaticRetryUsed = false;
            }
            progressAtRetry = progress;
            if (!resumeMotion()) return false;
            continue;
        }
        if (movement.isBusy()) chassis.update(elapsed / 1000.0f);
    }

    if (stopRequested()) return false;
    if (movement.getStatus() != MoveStatus::kDone) {
        if (centering) return finishCenteringEarly("Centrado parcial: tiempo limite alcanzado.");
        if (turning) {
            Serial.printf("Giro medido por BNO: %.1f de %.1f grados.\n", turned, targetTurn);
            return halt("Paro: tiempo limite del giro.");
        }
        return halt("Paro: tiempo limite de avance.");
    }
    if (turning) {
        Serial.printf("Giro completado por BNO: %.1f grados.\n", turned);
        if (chassis.hasEncoderOnEachSide()) {
            Serial.printf("Referencia de encoders: %.1f grados.\n", chassis.getTurnDegrees());
        }
    }
    if (completedTravel != nullptr) *completedTravel = traveled;
    if (kind == MazeMotion::kAdvance) Serial.printf("Avance completado. Encoders: %.1f cm.\n", encoderTravelCm());
    return true;
}

bool face(uint8_t direction, TurnRangeCapture &capture) {
    if (stopRequested()) return false;
    if (direction == facing) return true;
    int8_t steps = (direction + 4 - facing) % 4;
    if (steps == 3) steps = -1;
    float heading = 0.0f;
    if (!readHeading(heading)) return halt("Paro: sin rumbo del BNO para iniciar un giro.");
    const float correction = angleError(facing * 90.0f, heading);
    if (fabsf(correction) > kMaxHeadingError) return halt("Paro: rumbo fuera de la cuadricula.");
    const float turn = steps * 90.0f + correction;
    Serial.printf("Giro %d grados.\n", int(steps) * 90);
    const uint32_t timeout = steps == 2 ? kHalfTurnTimeoutMs : Motion::kMoveTimeoutMs;
    movement.startTurn(turn, timeout);
    if (!runMotion(MazeMotion::kTurn, heading, turn, {}, false, nullptr, &capture)) return false;
    facing = direction;
    return true;
}

float tileCenterCorrectionMm(const WallReference &front, const WallReference &back) {
    const bool nearFront = front.sensorId != VlxID::kNone && front.initialMm < MazeConfig::kFrontWallDistanceMm;
    const bool nearBack = back.sensorId != VlxID::kNone && back.initialMm < MazeConfig::kBackWallDistanceMm;
    const int frontCorrection = int(front.initialMm) - MazeConfig::kCenterFrontMm;
    const int backCorrection = int(MazeConfig::kCenterBackMm) - back.initialMm;
    if (nearFront && abs(frontCorrection) <= MazeConfig::kCenterToleranceMm) return 0.0f;
    if (nearFront && nearBack) return (frontCorrection + backCorrection) / 2.0f;
    if (nearFront) return frontCorrection;
    if (nearBack) return backCorrection;
    return 0.0f;
}

bool centerTile(RangeObservation (&observations)[VlxID::kCount]) {
    if (!chassis.hasEncoderOnEachSide()) return true;
    const auto front = wallReferenceOnSide(VlxID::kFront, observations);
    const auto back = wallReferenceOnSide(VlxID::kBack, observations);
    const float correctionMm = tileCenterCorrectionMm(front, back);
    if (fabsf(correctionMm) <= MazeConfig::kCenterToleranceMm) return true;

    const bool reversing = correctionMm < 0.0f;
    const uint8_t leadingSide = reversing ? VlxID::kBack : VlxID::kFront;
    uint16_t clearanceMm = 0;
    const VlxReading clearance = readSide(leadingSide, observations, clearanceMm);
    if (clearance == VlxReading::kUnknown ||
        (clearance == VlxReading::kDistance && clearanceMm - fabsf(correctionMm) < MazeConfig::kFrontStopMm)) {
        Serial.println("Centrado omitido: no hay espacio confirmado para el ajuste.");
        return true;
    }
    if (!ensureOrigin()) return false;

    const WallReference &leading = reversing ? back : front;
    const WallReference &trailing = reversing ? front : back;
    const auto &reference = leading.sensorId != VlxID::kNone && leading.initialMm < MazeConfig::wallDistanceMm(leadingSide) ?
        leading : trailing;
    Serial.printf("Centrado de casilla: %+.1f mm.\n", correctionMm);
    movement.startAdvance(correctionMm / 10.0f, Motion::kMoveTimeoutMs,
                          MazeConfig::kCenterToleranceMm / 10.0f, Motion::kMinSpeedCmS);
    if (!runMotion(MazeMotion::kCentering, 0.0f, 0.0f, reference, reversing)) return false;

    RangeObservation refreshed[VlxID::kCount]{};
    if (!captureRanges(kAllSides, refreshed)) return false;
    for (uint8_t i = 0; i < VlxID::kCount; ++i) {
        if (refreshed[i].reading != VlxReading::kUnknown) observations[i] = refreshed[i];
    }
    Serial.println("Fin del ajuste. Continuando DFS.");
    return true;
}

bool observeTile(RangeObservation (&observations)[VlxID::kCount]) {
    if (!captureRanges(kAllSides, observations)) return false;
    if (!centerTile(observations)) return false;
    uint8_t observed = 0;
    uint8_t open = 0;
    const uint8_t initialDirection = facing;
    for (uint8_t side = 0; side < VlxID::kSideCount; ++side) {
        const WallState wall = wallOnSide(side, observations, true);
        if (wall != WallState::kUnknown) {
            const uint8_t bit = 1u << ((initialDirection + side) % 4);
            observed |= bit;
            if (wall == WallState::kOpen) open |= bit;
        }
        if (stopRequested()) return false;
    }

    if (observed == 0) return halt("Paro: ningun lado dio una lectura valida.");
    if (!maze.observe(open, observed)) return halt("Paro: mapa lleno.");
    Serial.printf("Casilla (%d,%d), medidas 0x%X, salidas 0x%X, paredes 0x%X.\n",
                  int(maze.x()), int(maze.y()), unsigned(observed),
                  unsigned(maze.knownOpenDirections()), unsigned(maze.knownWallDirections()));
    return true;
}

bool exploreStep() {
    if (stopRequested()) return false;
    RangeObservation tileObservations[VlxID::kCount]{};
    if (!observeTile(tileObservations)) return false;

    constexpr float advanceMm = Geometry::kTileSizeCm * 10.0f;
    uint8_t unconfirmedDirections = 0;
    bool tileObservationsCurrent = true;
    while (!stopRequested()) {
        uint8_t direction = 0;
        const auto step = maze.next(direction, unconfirmedDirections);
        if (step == RescueMaze::Step::kDone) return halt("DFS terminado en la casilla inicial: sin salidas alcanzables.");
        if (step == RescueMaze::Step::kNoRoute) {
            return halt(unconfirmedDirections != 0 ? "Paro: no quedan salidas confirmables en esta casilla." :
                        "Paro: el DFS no encontro una ruta conocida.");
        }
        const uint8_t side = (direction + 4 - facing) % 4;
        WallState exitWall = tileObservationsCurrent ? wallOnSide(side, tileObservations) : WallState::kUnknown;
        if (exitWall == WallState::kUnknown) {
            RangeObservation probe[VlxID::kCount]{};
            auto &current = tileObservationsCurrent ? tileObservations : probe;
            if (!captureRanges(1u << side, current)) return false;
            exitWall = wallOnSide(side, current, true);
        }
        if (exitWall == WallState::kUnknown) {
            unconfirmedDirections |= 1u << direction;
            Serial.println("Salida sin confirmar desde esta posicion. Buscando otra ruta.");
            continue;
        }
        if (exitWall == WallState::kWall) {
            Serial.println("Pared detectada antes del giro. Buscando otra ruta.");
            if (!maze.observe(0, 1u << direction)) return halt("Paro: mapa lleno.");
            continue;
        }
        if (side == VlxID::kRight || side == VlxID::kLeft) {
            Serial.printf("Salida confirmada con VLX %s.\n", side == VlxID::kRight ? "derecho" : "izquierdo");
        }
        TurnRangeCapture turnRanges{};
        if (!ensureOrigin() || !face(direction, turnRanges)) return false;
        if (side != VlxID::kFront) tileObservationsCurrent = false;

        if (!tileObservationsCurrent) {
            if (!turnRanges.started && !startRangeCapture(kWallReferenceSides, turnRanges.pending)) return false;
            if (!finishRangeCapture(turnRanges.pending, turnRanges.observations)) return false;
        }
        const auto &observations = tileObservationsCurrent ? tileObservations : turnRanges.observations;
        uint16_t distance = 0;
        const VlxReading front = readSide(VlxID::kFront, observations, distance, !tileObservationsCurrent);
        if (stopRequested()) return false;
        if (wallOnSide(VlxID::kFront, observations) == WallState::kWall ||
            (front == VlxReading::kDistance && distance < MazeConfig::kFrontStopMm)) {
            Serial.println("El frontal detecto un obstaculo. Buscando otra ruta.");
            if (!maze.observe(0, 1u << direction)) return halt("Paro: mapa lleno.");
            continue;
        }

        WallReference reference = wallReferenceOnSide(VlxID::kFront, observations);
        if (reference.sensorId == VlxID::kNone) reference = wallReferenceOnSide(VlxID::kBack, observations);
        if (reference.sensorId != VlxID::kNone) {
            const float targetWallMm = reference.behind ? reference.initialMm + advanceMm :
                max(float(MazeConfig::kCenterFrontMm), reference.initialMm - advanceMm);
            Serial.printf("Referencia %s VLX%u: %u -> %.0f mm.\n", reference.behind ? "trasera" : "frontal",
                          unsigned(reference.sensorId), unsigned(reference.initialMm), targetWallMm);
        }
        Serial.printf("Avance %.1f cm.\n", Geometry::kTileSizeCm);
        movement.startAdvance(Geometry::kTileSizeCm, Motion::kMoveTimeoutMs, 0.0f, MazeConfig::kTravelSpeedCmS);
        float traveled = 0.0f;
        if (!runMotion(MazeMotion::kAdvance, 0.0f, 0.0f, reference, false, &traveled)) return false;
        if (traveled < Geometry::kTileSizeCm / 2.0f) {
            tileObservationsCurrent = false;
            Serial.println("Pared alcanzada dentro de la casilla. Buscando otra ruta.");
            if (!maze.observe(0, 1u << direction)) return halt("Paro: mapa lleno.");
            continue;
        }
        if (!maze.arrive(direction)) return halt("Paro: el avance no coincide con el mapa.");
        return true;
    }
    return false;
}
}

void setup() {
    chassis.begin();
    chassis.stop();
    movement.begin(&chassis);
    Serial.begin(Comms::kUsbBaudRate);

    Wire.setSDA(Pins::kI2cSda);
    Wire.setSCL(Pins::kI2cScl);
    Wire.begin();
    Wire.setClock(100000);
    Wire1.setSDA(Pins::kI2c1Sda);
    Wire1.setSCL(Pins::kI2c1Scl);
    Wire1.begin();
    Wire1.setClock(100000);
    if (stopRequested()) return;
    if (!bno.begin()) {
        halt("Paro: el BNO no responde en Wire1.");
        return;
    }
    if (!initRanges()) return;

    Serial.println("DFS en 3 segundos. Envia x por USB para parar.");
    const uint32_t start = millis();
    while (millis() - start < kStartupDelayMs) {
        if (stopRequested()) return;
        OrientationQuaternion sample{};
        if (bno.readQuaternion(sample)) {
            origin = sample;
            haveOrigin = true;
        }
        delay(Control::kPeriodMs);
    }
    Serial.println("DFS iniciado.");
    exploring = true;
}

void loop() {
    if (exploring) exploring = exploreStep();
    else delay(10);
}
