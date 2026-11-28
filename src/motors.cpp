#include "motors.h"
#include "TileAdvance.h"
#include "Encoder.h"
#include "Pins_ID.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
motors::motors() {}

void motors::setupMotors(bool initializeMazeSensors) {
  Wire.setSDA(Pins::i2cSda);
  Wire.setSCL(Pins::i2cScl);
  Wire.begin();
  Wire1.setSDA(Pins::i2c1Sda);
  Wire1.setSCL(Pins::i2c1Scl);
  Wire1.begin();
  Wire.setClock(Comms::kI2cClockHz);
  Wire1.setClock(Comms::kI2cClockHz);
  bno.setupBNO();
  if (initializeMazeSensors) {
    resetVlx();
    setupTCS();
  } else {
    bno.resetOrientation();
  }

  for (uint8_t i = 0; i < 4; i++) {
    motor[i].initialize(Pins::digitalOne[i], Pins::digitalTwo[i],
                        Pins::pwmPin[i], i);
    myPID[i].changeConstants(kP_mov, kI_mov, kD_mov, movTime);
  }
  rampUpPID.changeConstants(kP_RampUp, kI_RampUp, kD_RampUp, rampTime);
  rampDownPID.changeConstants(kP_RampDown, kI_RampDown, kD_RampDown, rampTime);

  targetAngle = 0;
  delay(Startup::kSettleMs);
}
void motors::printAngle() {
  double angulo = bno.getOrientationX();
  Serial.println(angulo);
}

bool motors::checkEncoderHealth() {
  const uint32_t now = millis();
  for (uint8_t wheel = 0; wheel < MotorID::kCount; ++wheel) {
    if (encoderHealth_.observe(wheel, motor[wheel].getTotalEncoderTics(),
                               motor[wheel].getSpeed(), now)) {
      Serial.print("Encoder signal lost on wheel slot ");
      Serial.println(wheel);
    }
  }
  if (encoderHealth_.hasFeedback()) return true;
  if (inMotion) Serial.println("Stopped: no encoder feedback.");
  stop();
  return false;
}

void motors::PID_Wheel(float targetSpeed, int i) {
  if (!checkEncoderHealth()) return;
  const uint8_t partner = EncoderHealth::partner(i);
  if (encoderHealth_.failed(i)) {
    if (encoderHealth_.failed(partner)) {
      const float fraction = constrain(targetSpeed / kMaxSpeedForward, 0.0f, 1.0f);
      const int pwm = targetSpeed > 0.0f ?
          Motion::kNoEncoderMinPwm + fraction * (Motion::kNoEncoderMaxPwm - Motion::kNoEncoderMinPwm) : 0;
      motor[i].setSpeed(pwm);
    }
    return;
  }
  const float speed_setpoint = targetSpeed;
  int reference_pwm = motor[i].getSpeed();
  const float speedTics = fabsf(motor[i].getTicsSpeed());
  if (Control::kDebugPid) {
    Serial.print("reference_pwm: ");
    Serial.println(reference_pwm);
    Serial.print("speedTics: ");
    Serial.println(speedTics);
  }
  float error = myPID[i].calculate_PID(speed_setpoint, speedTics);
  int speed = reference_pwm + error;
  speed = constrain(speed, 0, static_cast<int>(Control::kPwmMax));
  motor[i].setSpeed(speed);
  if (encoderHealth_.failed(partner)) motor[partner].setSpeed(speed);
}
void motors::PID_AllWheels(float targetSpeed) {
  for (uint8_t i = 0; i < 4; i++) {
    PID_Wheel(targetSpeed, i);
    if (!encoderHealth_.hasFeedback()) break;
  }
}

void motors::pidEncoders(float speedReference, bool ahead) {
  const bool headingValid = bno.tryReadHeading(estimatedHeading_);
  static PID pidBno(Control::kHeadingKp, Control::kHeadingKi, Control::kHeadingKd,
                    Control::kHeadingPidPeriodMs);
  if (rampState != 0)
    changeAngle = 0;
  const float headingError = remainderf(targetAngle + changeAngle - estimatedHeading_, 360.0f);
  float AngleError = headingValid ? pidBno.calculate_PID(headingError, 0.0f) : 0.0f;
  AngleError = constrain(AngleError, -Control::kHeadingCorrectionMax, Control::kHeadingCorrectionMax);
  if (Control::kDebugPid) Serial.println(AngleError);
  if (!ahead)
    AngleError = -AngleError;
  PID_Wheel(speedReference + AngleError, MotorID::kFrontLeft);
  PID_Wheel(speedReference + AngleError, MotorID::kBackLeft);
  PID_Wheel(speedReference - AngleError, MotorID::kFrontRight);
  PID_Wheel(speedReference - AngleError, MotorID::kBackRight);
}

namespace {
float advancePidTarget(float remainingCm, bool rampCaution) {
  const float maxSpeed = rampCaution ? MazeConfig::kTravelSpeedCmS / 3.0f
                                     : MazeConfig::kTravelSpeedCmS;
  return Control::toWheelPidTicks(constrain(Motion::kDecelGain * remainingCm,
                                           Motion::kMinSpeedCmS, maxSpeed));
}
}

void motors::ahead() {
  blueTile = false;
  checkpoint = false;
  // Double check before starting the tile.
  if (buttonPressed || !passObstacle() || !passObstacle()) return;
  nearWall();
  resetTics();

  using Reference = TileAdvance::Reference;
  const auto readReference = [this](uint8_t slot, uint16_t maxMm, float &cm, uint32_t *sampleTimeMs = nullptr) {
    uint16_t mm = 0;
    if (!vlx[slot].readValidMillimeters(mm, sampleTimeMs) || mm < 10 || mm >= maxMm) return false;
    cm = mm / 10.0f;
    return true;
  };

  constexpr uint16_t maxFrontMm = MazeConfig::kWallReferenceMaxMm;
  constexpr uint16_t maxInitialBackMm = maxFrontMm - Geometry::kTileSizeCm * 10;
  uint16_t frontLeftMm = 0, frontRightMm = 0;
  const bool frontLeftRead = vlx[vlxID::frontLeft].readValidMillimeters(frontLeftMm);
  const bool frontRightRead = vlx[vlxID::frontRight].readValidMillimeters(frontRightMm);
  const bool frontLeftValid = frontLeftRead && frontLeftMm >= 10 && frontLeftMm < maxFrontMm;
  const bool frontRightValid = frontRightRead && frontRightMm >= 10 && frontRightMm < maxFrontMm;
  const bool rampCaution = !frontLeftRead || !frontRightRead;
  float wallCm = 0.0f;
  Reference reference = Reference::kEncoder;
  uint8_t wallSlot = vlxID::backLeft;
  float wallTargetCm = 0.0f;
  float encoderTargetCm = kTileLength;

  const bool startsOnSlope = abs(bno.getOrientationY()) > Motion::kStartOnSlopeAngleDegrees;
  if (startsOnSlope) {
    // Preserve the original shortened encoder step when already on a slope.
    encoderTargetCm -= kTileLength / 6.0f;
  } else if (frontLeftValid || frontRightValid) {
    reference = Reference::kFront;
    const bool useLeft = frontLeftValid && (!frontRightValid || frontLeftMm >= frontRightMm);
    wallSlot = useLeft ? vlxID::frontLeft : vlxID::frontRight;
    wallCm = (useLeft ? frontLeftMm : frontRightMm) / 10.0f;
    wallTargetCm = findNearest(wallCm, targetDistances, 2, true);
  } else if (readReference(wallSlot, maxInitialBackMm, wallCm)) {
    reference = Reference::kBack;
    wallTargetCm = findNearest(wallCm, targetDistancesB, 2, false);
  }

  float lastWallEncoderCm = 0.0f;
  TileAdvance advance(reference, wallTargetCm, encoderTargetCm);
  FrontWallCandidate candidate;
  uint32_t lastFrontProbeMs = millis();
  float remaining = advance.remainingCm(getCurrentDistanceCm(), true, wallCm);
  while (!advance.done(remaining) && checkEncoderHealth()) {
    setahead();
    checkTileColor();
    if (blackTile || buttonPressed || isRamp()) break;
    if (slope && getCurrentDistanceCm() >= encoderTargetCm) break;

    pidEncoders(advancePidTarget(remaining, rampCaution), true);

    // Keep the same sensor while it still sees the original wall.
    // Encoder counts have been accumulating since the start of this tile.
    bool validWall = true;
    if (advance.reference() != Reference::kEncoder) {
      const auto sampleWall = [&](float &sampleCm) {
        if (!readReference(wallSlot, maxFrontMm, sampleCm)) return false;
        // A sudden jump away from the expected range usually means another wall.
        const float traveled = getCurrentDistanceCm() - lastWallEncoderCm;
        const float expected = wallCm +
            (advance.reference() == Reference::kFront ? -traveled : traveled);
        return fabsf(sampleCm - expected) <= MazeConfig::kWallReferenceJumpMm / 10.0f;
      };
      float sampleCm = 0.0f;
      validWall = sampleWall(sampleCm);
      if (validWall) {
        wallCm = sampleCm;
        lastWallEncoderCm = getCurrentDistanceCm();
      } else {
        Serial.println("Wall reference lost; finishing tile by encoders.");
      }
    }
    remaining = advance.remainingCm(getCurrentDistanceCm(), validWall, wallCm);
    const uint32_t now = millis();
    if (advance.canAdoptFront() && !advance.done(remaining) &&
        !startsOnSlope && !slope && now - lastFrontProbeMs >= MazeConfig::kFrontWallProbeMs) {
      lastFrontProbeMs = now;
      float leftCm = 0.0f, rightCm = 0.0f;
      uint32_t leftSampleMs = 0, rightSampleMs = 0;
      const bool haveLeft = readReference(vlxID::frontLeft, maxFrontMm, leftCm, &leftSampleMs);
      const bool haveRight = readReference(vlxID::frontRight, maxFrontMm, rightCm, &rightSampleMs);
      const float encoderCm = getCurrentDistanceCm();
      remaining = advance.remainingCm(encoderCm, true, wallCm);
      if (haveLeft || haveRight) {
        const bool pickLeft = haveLeft && (!haveRight || leftCm >= rightCm);
        const uint8_t candidateSlot = pickLeft ? vlxID::frontLeft : vlxID::frontRight;
        const float candidateCm = pickLeft ? leftCm : rightCm;
        const uint32_t sampleTimeMs = pickLeft ? leftSampleMs : rightSampleMs;
        if (!advance.done(remaining) &&
            candidate.observe(candidateSlot, candidateCm, encoderCm, sampleTimeMs) &&
            advance.adoptFront(encoderCm, candidateCm)) {
          wallSlot = candidateSlot;
          wallCm = candidateCm;
          lastWallEncoderCm = encoderCm;
          remaining = advance.remainingCm(encoderCm, true, wallCm);
          Serial.println("Front wall confirmed; using it to finish this tile.");
        }
      } else {
        candidate.reset();
      }
    }
  }
  slope = false;
  stop();
  if (!rampRealignmentPending_ && !blackTile && !buttonPressed && encoderHealth_.hasFeedback())
    checkTileColor(true);
  resetTics();
}

void motors::checkTileColor(bool atEnd) {
  if (!floorColorReady_ || buttonPressed) return;
  const uint32_t now = millis();
  if (!atEnd && now - lastFloorColorSampleMs_ < MazeConfig::kFloorColorSampleMs) return;
  lastFloorColorSampleMs_ = now;
  const char tileColor = tcs_.getColor();
  if (tileColor == kBlackColor) {
    stop();
    const long returnTics = getAverageTicks();
    blackTile = true;
    Serial.println("Black Tile");
    resetTics();
    const uint32_t retreatStart = millis();
    while (getAverageTicks() < returnTics && !buttonPressed &&
           checkEncoderHealth() && millis() - retreatStart < MazeConfig::kBlackTileRetreatTimeoutMs) {
      setback();
      const float remainingCm = (returnTics - getAverageTicks()) * Geometry::kCmPerRisingTick;
      pidEncoders(advancePidTarget(remainingCm, false), false);
    }
    stop();
    if (!buttonPressed && getAverageTicks() < returnTics) {
      Serial.println("Stopped: could not return from black tile.");
      while (true) delay(50);
    }
    resetTics();
  } else if (tileColor == kBlueColor && atEnd) {
    // screenPrint("Blue Tile");
    Serial.println("Blue Tile");
    blueTile = true;
    wait(MazeConfig::kBlueTileWaitMs);
  } else if (tileColor == kCheckpointColor && atEnd) {
    checkpoint = true;
    Serial.println("Checkpoint Tile");
    // screenPrint("Checkpoint Tile");
  }
}

float motors::nearWall() {
  constexpr uint16_t clearanceMm = MazeConfig::kLateralClearanceCm * 10;
  if (vlx[vlxID::leftFront].readWall(clearanceMm) == WallReading::kWall) {
    changeAngle = maxChangeAngle;
  } else if (vlx[vlxID::rightFront].readWall(clearanceMm) == WallReading::kWall) {
    changeAngle = -maxChangeAngle;
  } else {
    changeAngle = 0;
  }
  return changeAngle;
}

bool motors::passObstacle() {
  const float originalHeading = targetAngle;
  bno.tryReadHeading(estimatedHeading_);
  const bool needsAlignment = rampRealignmentPending_ ||
      fabsf(remainderf(originalHeading - estimatedHeading_, 360.0f)) >= 2.0f;
  if (needsAlignment && !rotate(originalHeading)) return false;

  const WallReading left = vlx[vlxID::frontLeft].readWall(kObstacleDistanceMm);
  const WallReading right = vlx[vlxID::frontRight].readWall(kObstacleDistanceMm);
  // A detour needs a confirmed obstacle and a confirmed open side.
  if (left == WallReading::kUnknown || right == WallReading::kUnknown || left == right)
    return true;
  const bool leftBlocked = left == WallReading::kWall;

  moveDistance(Motion::kObstacleRetreatCm, false);
  limitColition = true;
  const float sideHeading = originalHeading +
      (leftBlocked ? Motion::kObstacleTurnDegrees : -Motion::kObstacleTurnDegrees);
  const bool turnedAside = rotate(sideHeading);
  if (turnedAside) moveDistance(Motion::kObstaclePassCm, true);
  const bool restored = turnedAside && rotate(originalHeading);
  limitColition = false;
  targetAngle = originalHeading;
  return restored;
}
/*
void motors::limitCrash(){
    float targetAngle_=targetAngle;
    bool leftState=limitSwitch_[LimitSwitchID::kLeft].getState();
    bool rightState=limitSwitch_[LimitSwitchID::kRight].getState();
    if(slope) return;
    if(rampState!=0 ){
        if(leftState || rightState) limitColition=true;
        return;
    }
    if((leftState && rightState) || (!leftState && !rightState)){
        return;
    }
    else if(rightState){
        screenPrint("RightLimit");
        Serial.println("rightlimit");
        if(targetAngle==360){
            targetAngle=0;
        }
        rotate(targetAngle+25);
    }else if(leftState){
        screenPrint("leftLimit");
        Serial.println("rightlimit");
        if(targetAngle==0){
            targetAngle=360;
        }
        rotate(targetAngle-25);
    }
    delay(300);
    moveDistance(kTileLength/6,false);
    delay(300);
    targetAngle=targetAngle_;
    rotate(targetAngle);
    limitColition=false;
}
*/
float motors::findNearest(float number, const float numbers[], uint8_t size,
                            bool frontVlx) {
  if (frontVlx)
    number -= kTileLength;
  else if (!frontVlx)
    number += kTileLength;
  float nearest = numbers[0];
  float minDifference = abs(number - numbers[0]);
  for (uint8_t i = 1; i < size; i++) {
    float currentDifference = abs(number - numbers[i]);
    if (currentDifference < minDifference) {
      nearest = numbers[i];
      minDifference = currentDifference;
    }
  }
  return nearest;
}
void motors::back() { setback(); }
bool motors::right() {
  Serial.println("right");
  return rotate(targetAngle + 90.0f);
}
bool motors::left() {
  Serial.println("left");
  return rotate(targetAngle - 90.0f);
}

float motors::calculateAngularDistance() {
  bno.tryReadHeading(estimatedHeading_);
  return remainderf(targetAngle - estimatedHeading_, 360.0f);
}
float motors::encoderTurnDegrees(const int *startTicks) {
  int ticks[MotorID::kCount];
  for (uint8_t wheel = 0; wheel < MotorID::kCount; ++wheel)
    ticks[wheel] = motor[wheel].getTics() - startTicks[wheel];
  const float difference = encoderHealth_.leftAverage(ticks) - encoderHealth_.rightAverage(ticks);
  const float degrees = difference * Geometry::kCmPerRisingTick * 180.0f /
      (Geometry::kPi * Geometry::kTrackWidthCm);
  return Motion::scaleEncoderTurn(degrees);
}

bool motors::rotateByEncoders(float degrees) {
  return rotate(estimatedHeading_ + degrees, TurnFeedback::kEncodersOnly);
}

bool motors::rotate(float headingDegrees, TurnFeedback feedback) {
  const float previousTarget = targetAngle;
  targetAngle = fmodf(headingDegrees, 360.0f);
  if (targetAngle < 0.0f) targetAngle += 360.0f;

  int anchorTicks[MotorID::kCount];
  for (uint8_t wheel = 0; wheel < MotorID::kCount; ++wheel)
    anchorTicks[wheel] = motor[wheel].getTics();
  float anchorHeading = estimatedHeading_;
  uint32_t anchorReportMs = 0;
  bool hasAnchorReport = false;
  const uint32_t startedMs = millis();
  bool reachedTarget = false;
  bool usingEncoders = feedback == TurnFeedback::kEncodersOnly;
  int turnDirection = 0;
  uint32_t timeoutMs = Motion::kTurnTimeoutMs;
  while (!buttonPressed && millis() - startedMs < timeoutMs &&
         checkEncoderHealth()) {
    uint32_t reportMs;
    const bool hasHeading = !usingEncoders && bno.tryReadHeading(estimatedHeading_, &reportMs);
    if (hasHeading) {
      if (!hasAnchorReport || reportMs != anchorReportMs) {
        hasAnchorReport = true;
        anchorReportMs = reportMs;
        anchorHeading = estimatedHeading_;
        for (uint8_t wheel = 0; wheel < MotorID::kCount; ++wheel)
          anchorTicks[wheel] = motor[wheel].getTics();
      }
    } else {
      if (feedback == TurnFeedback::kBnoOnly) {
        Serial.println("Giro interrumpido: se perdio el rumbo BNO.");
        break;
      }
      if (!encoderHealth_.hasFeedbackOnBothSides()) {
        Serial.println("Giro interrumpido: se necesita un encoder funcional por lado.");
        break;
      }
      if (!usingEncoders) Serial.println("BNO unavailable. Turning with encoders.");
      usingEncoders = true;
      estimatedHeading_ = anchorHeading + encoderTurnDegrees(anchorTicks);
    }
    const float error = remainderf(targetAngle - estimatedHeading_, 360.0f);
    if (turnDirection == 0 && fabsf(error) >= Motion::kHalfTurnMinDegrees)
      timeoutMs = Motion::kHalfTurnTimeoutMs;
    const float tolerance = usingEncoders ? Motion::kEncoderTurnToleranceDegrees :
                                           Motion::kTurnToleranceDegrees;
    if (fabsf(error) <= tolerance) {
      reachedTarget = true;
      break;
    }
    const int direction = error > 0.0f ? 1 : -1;
    if (direction != turnDirection) {
      if (turnDirection != 0) stop();
      if (direction > 0) setright();
      else setleft();
      turnDirection = direction;
      inMotion = true;
    }
    const float speedCmS = constrain(Motion::kTurnSpeedGain * fabsf(error),
                                     Motion::kMinTurnSpeedCmS, Motion::kTurnSpeedCmS);
    PID_AllWheels(Control::toWheelPidTicks(speedCmS));
  }
  stop();
  if (!reachedTarget) {
    targetAngle = previousTarget;
    if (millis() - startedMs >= timeoutMs) {
      Serial.println("Stopped: turn timed out.");
    }
  }
  if (reachedTarget) rampRealignmentPending_ = false;
  return reachedTarget;
}

float motors::changeSpeedMove(bool encoders, bool rotate, int targetDistance,
                              bool frontVlx) {
  float speed;
  float missingDistance, missingAngle;
  if (rotate == true) {
    missingAngle = fabsf(calculateAngularDistance());
    speed = Control::toWheelPidTicks(constrain(Motion::kTurnSpeedGain * missingAngle,
                                               Motion::kMinTurnSpeedCmS, Motion::kTurnSpeedCmS));
    PID_AllWheels(speed);
    return 0;
  } else {
    if (encoders == true) {
      speed =
          map(getAverageTicks(), 0, kTicsPerTile, Control::kMaxForwardPwm, Control::kMinForwardPwm);
      missingDistance =
          kTileLength - (getAverageTicks() * kTileLength / kTicsPerTile);
      return speed;
    } else {
      missingDistance = abs((frontVlx ? vlx[vlxID::frontLeft].getDistance()
                                      : vlx[vlxID::backLeft].getDistance()) -
                            targetDistance); // intercambiar vlx
      speed = frontVlx ? map(missingDistance, kTileLength, 0, Control::kMaxForwardPwm,
                             Control::kMinForwardPwm)
                       : map(missingDistance, kTileLength, 0, Control::kMaxForwardPwm,
                             Control::kMinForwardPwm);
      speed = constrain(speed, Control::kMinForwardPwm, Control::kMaxForwardPwm);
      return speed;
    }
  }
}

void motors::setSpeed(uint16_t speed) {
  if (speed != 0 && !checkEncoderHealth()) return;
  for (uint8_t i = 0; i < 4; i++) {
    motor[i].setSpeed(speed);
  }
}
void motors::setahead() {
  for (int i = 0; i < 4; i++) {
    motor[i].ahead();
  }
  inMotion = true;
}
void motors::setback() {
  for (int i = 0; i < 4; i++) {
    motor[i].back();
  }
  // inMotion=true;
}
void motors::setright() {
  motor[MotorID::kBackLeft].ahead();
  motor[MotorID::kBackRight].back();
  motor[MotorID::kFrontLeft].ahead();
  motor[MotorID::kFrontRight].back();
  // inMotion=true;
}
void motors::setleft() {
  motor[MotorID::kBackLeft].back();
  motor[MotorID::kBackRight].ahead();
  motor[MotorID::kFrontLeft].back();
  motor[MotorID::kFrontRight].ahead();
  // inMotion=true;
}
void motors::setrightTraslation() {
  motor[MotorID::kBackLeft].stop();
  motor[MotorID::kBackRight].back();
  motor[MotorID::kFrontLeft].stop();
  motor[MotorID::kFrontRight].back();
  // inMotion=true;
}
void motors::setleftTraslation() {
  motor[MotorID::kBackLeft].back();
  motor[MotorID::kBackRight].stop();
  motor[MotorID::kFrontLeft].back();
  motor[MotorID::kFrontRight].stop();
  // inMotion=true;
}

void motors::stop() {
  for (uint8_t i = 0; i < 4; i++) {
    motor[i].stop();
  }
  setSpeed(0);
  encoderHealth_.endMotion();
  inMotion = false;
}
void motors::printSpeeds() {
  float speedM1 = motor[0].getSpeed();
  float speedM2 = motor[1].getSpeed();
  float speedM3 = motor[2].getSpeed();
  float speedM4 = motor[3].getSpeed();
  Serial.println("speed:");
  Serial.print("Motor1:");
  Serial.print(speedM1);
  Serial.print("Motor2:");
  Serial.print(speedM2);
  Serial.print("Motor3:");
  Serial.print(speedM3);
  Serial.print("Motor4:");
  Serial.println(speedM4);
}
void motors::resetTics() {
  for (uint8_t i = 0; i < 4; i++) {
    motor[i].resetTics();
  }
}
double motors::getAverageTicks() {
  checkEncoderHealth();
  int counts[MotorID::kCount];
  for (uint8_t wheel = 0; wheel < MotorID::kCount; ++wheel)
    counts[wheel] = abs(motor[wheel].getTics());
  return encoderHealth_.average(counts);
}
double motors::getTicsSpeed() {
  checkEncoderHealth();
  float speeds[MotorID::kCount];
  for (uint8_t wheel = 0; wheel < MotorID::kCount; ++wheel)
    speeds[wheel] = fabsf(motor[wheel].getTicsSpeed());
  return encoderHealth_.average(speeds);
}
void motors::setupVlx(const uint8_t index) {
  if (vlx[index].begin(vlxID::connectors[index])) return;
  Serial.print("VLX failed at connector ");
  const uint8_t connector = vlxID::connectors[index];
  Serial.println(connector);
  if (connector != Pins::vlxWithoutXshut) {
    // Keep a failed switched sensor off the bus before starting the next one.
    const uint8_t pin = Pins::vlxXshut[connector - 1];
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
}
void motors::resetVlx() {
  if (!VLX::prepareBus()) {
    Serial.println("VLX: failed to isolate the sensor without XSHUT.");
    while (true) delay(50);
  }
  constexpr uint8_t slots[] = {
      vlxID::frontRight, vlxID::rightFront, vlxID::frontLeft, vlxID::backLeft, vlxID::leftFront,
      vlxID::rightBack, vlxID::leftBack, vlxID::backRight,
  };
  // Keep 0x29 free until the switched sensors have unique addresses.
  constexpr uint8_t noSlot = sizeof(vlxID::connectors);
  uint8_t unswitchedSlot = noSlot;
  for (uint8_t slot : slots) {
    const uint8_t connector = vlxID::connectors[slot];
    if (connector == vlxID::kNone) continue;
    if (connector == Pins::vlxWithoutXshut)
      unswitchedSlot = slot;
    else
      setupVlx(slot);
  }
  if (unswitchedSlot != noSlot) setupVlx(unswitchedSlot);
}

/*
float motors::AverageLeftDistance(){
    float frontLeftDistance=vlx[vlxID::frontLeft].getDistance();
    float leftDistance=vlx[vlxID::leftFront].getDistance();

    if (frontLeftDistance <= maxVlxDistance && leftDistance <= maxVlxDistance) {
        return (frontLeftDistance+leftDistance)/2;
    } else if (frontLeftDistance > maxVlxDistance && leftDistance <=
maxVlxDistance) { return leftDistance; } else if (frontLeftDistance <=
maxVlxDistance && leftDistance > maxVlxDistance) { return frontLeftDistance;
    }
    else {
        return maxVlxDistance;
    }
}
float motors::AverageRightDistance(){
    float rightDownDistance=vlx[vlxID::rightDown].getDistance();
    float rightDistance=vlx[vlxID::rightFront].getDistance();

    if (rightDownDistance <= maxVlxDistance && rightDistance <= maxVlxDistance)
{ return (rightDownDistance+rightDistance)/2; } else if (rightDownDistance >
maxVlxDistance && rightDistance <= maxVlxDistance) { return rightDistance; }
else if (rightDownDistance <= maxVlxDistance && rightDistance > maxVlxDistance)
{ return rightDownDistance;
    }
    else {
        return maxVlxDistance;
    }
}
*/
WallReading motors::readWall(uint8_t direction) {
  uint8_t relativeDir = 0;
  int deltaTargetAngle = static_cast<int>(targetAngle);
  switch (deltaTargetAngle) {
  case 0:
  case 360:
    relativeDir = 0;
    break;
  case 90:
    relativeDir = 1;
    break;
  case 180:
    relativeDir = 2;
    break;
  case 270:
    relativeDir = 3;
    break;
  default:
    // Unexpected orientation, keep default (front)
    relativeDir = 0;
    break;
  }
  constexpr uint8_t slotsBySide[][2] = {
      {vlxID::frontLeft, vlxID::frontRight},
      {vlxID::rightFront, vlxID::rightBack},
      {vlxID::backLeft, vlxID::backRight},
      {vlxID::leftFront, vlxID::leftBack},
  };
  if (direction >= 4) return WallReading::kUnknown;
  const uint8_t robotSide = rulet[relativeDir][direction];
  const auto &slots = slotsBySide[robotSide];
  const uint16_t wallDistance = robotSide == 0 ? MazeConfig::kFrontWallDistanceMm :
                                robotSide == 2 ? MazeConfig::kBackWallDistanceMm :
                                                 MazeConfig::kSideWallDistanceMm;
  const bool firstInstalled = vlx[slots[0]].initialized;
  const bool secondInstalled = vlx[slots[1]].initialized;

  const WallReading first = firstInstalled ? vlx[slots[0]].readWall(wallDistance)
                                             : WallReading::kUnknown;
  const WallReading second = secondInstalled ? vlx[slots[1]].readWall(wallDistance)
                                             : WallReading::kUnknown;
  if (firstInstalled && secondInstalled) {
    if (first == second) return first;
    return WallReading::kUnknown;
  }
  return firstInstalled ? first : second;
}

bool motors::rampInFront() {
  uint16_t leftMm, rightMm;
  return vlx[vlxID::frontLeft].readValidMillimeters(leftMm) &&
         vlx[vlxID::frontRight].readValidMillimeters(rightMm) &&
         (static_cast<int>(leftMm) - rightMm) / 10.0f >= MazeConfig::kRampFrontDifferenceCm;
}
bool motors::isRamp() {
  float currentOrientationY = bno.getOrientationY();
  // screenPrint("isRamp");
  if (abs(currentOrientationY) > Motion::kSlopeAngleDegrees)
    slope = true;
  if (currentOrientationY >= kMinRampOrientation ||
      currentOrientationY <= -kMinRampOrientation) {

    if (currentOrientationY <= -kMinRampOrientation) {
      // screenPrint("Ramp detected");
      ramp();
      return true;
    } else if (currentOrientationY > kMinRampOrientation) {
      // screenPrint("Ramp detected");
      ramp();
      return true;
    }
  }
  return false;
}
void motors::ramp() {
  resetTics();
  setahead();
  while (bno.getOrientationY() > Motion::kSlopeAngleDegrees && checkEncoderHealth()) {
    if (buttonPressed == true)
      break;
    // limitCrash();
    if (limitColition == true) {
      stop();
      break;
    }
    float error;
    vlx[vlxID::rightFront].getDistance();
    vlx[vlxID::leftFront].getDistance();
    if ((vlx[vlxID::rightFront].distance < MazeConfig::kRampWallDistanceCm &&
         vlx[vlxID::leftFront].distance < MazeConfig::kRampWallDistanceCm) &&
        (vlx[vlxID::rightFront].distance < MazeConfig::kLateralClearanceCm || vlx[vlxID::leftFront].distance < MazeConfig::kLateralClearanceCm)) {
      error = rampUpPID.calculate_PID(
          0, (vlx[vlxID::rightFront].distance - vlx[vlxID::leftFront].distance));
      error = constrain(error, -Control::kRampUpCorrectionMax, Control::kRampUpCorrectionMax);
      PID_Wheel(kSpeedRampUp - error, MotorID::kFrontLeft);
      PID_Wheel(kSpeedRampUp - error, MotorID::kBackLeft);
      PID_Wheel(kSpeedRampUp + error, MotorID::kFrontRight);
      PID_Wheel(kSpeedRampUp + error, MotorID::kBackRight);
    } else {
      pidEncoders(kSpeedRampUp, true);
    }
    rampState = 1;
    screenPrint("rampUp");
  }
  while (bno.getOrientationY() < -Motion::kSlopeAngleDegrees && checkEncoderHealth()) {
    if (buttonPressed == true)
      break;
    // limitCrash();
    if (limitColition == true)
      break;
    float error;
    vlx[vlxID::rightFront].getDistance();
    vlx[vlxID::leftFront].getDistance();
    if ((vlx[vlxID::rightFront].distance < MazeConfig::kRampWallDistanceCm &&
         vlx[vlxID::leftFront].distance < MazeConfig::kRampWallDistanceCm) &&
        (vlx[vlxID::rightFront].distance < MazeConfig::kLateralClearanceCm || vlx[vlxID::leftFront].distance < MazeConfig::kLateralClearanceCm)) {
      error = rampDownPID.calculate_PID(
          0, (vlx[vlxID::rightFront].distance - vlx[vlxID::leftFront].distance));
      error = constrain(error, -Control::kRampDownCorrectionMax, Control::kRampDownCorrectionMax);
      PID_Wheel(kSpeedRampDown - error, MotorID::kFrontLeft);
      PID_Wheel(kSpeedRampDown - error, MotorID::kBackLeft);
      PID_Wheel(kSpeedRampDown + error, MotorID::kFrontRight);
      PID_Wheel(kSpeedRampDown + error, MotorID::kBackRight);
    } else {
      pidEncoders(kSpeedRampDown, true);
    }
    rampState = 2;
    // screenPrint("rampDown");
  }
  if (!isfinite(bno.getOrientationY()) || !encoderHealth_.hasFeedback()) {
    rampState = rampID::kNone;
    limitColition = false;
    stop();
    return;
  }
  const double traveledTicks = getAverageTicks();
  uint8_t exitDistanceCm = 0;
  if (rampState == rampID::kUp && traveledTicks > kTicsPerTile) {
    exitDistanceCm = kTileLength / 3;
  } else if (rampState == rampID::kDown && traveledTicks > 0.8 * kTicsPerTile) {
    exitDistanceCm = kTileLength / 2;
  }

  if (exitDistanceCm != 0) {
    moveDistance(exitDistanceCm, true);
    const float exitHeading = targetAngle;
    bool aligned = rotate(exitHeading);
    if (!aligned && !buttonPressed && encoderHealth_.hasFeedback()) {
      Serial.println("Salida de rampa: reintentando alinear el rumbo.");
      wait(100);
      if (!buttonPressed) aligned = rotate(exitHeading);
    }
    rampRealignmentPending_ = !aligned;
    if (rampRealignmentPending_) {
      Serial.printf("Salida de rampa: alineacion pendiente. Objetivo %.1f, rumbo estimado %.1f grados.\n",
                    exitHeading, estimatedHeading_);
    }
  }

  if (exitDistanceCm == 0 || rampRealignmentPending_) rampState = rampID::kNone;
  limitColition = false;
  resetTics();
  stop();
  if (!rampRealignmentPending_) wait(100);
}

void motors::moveDistance(uint8_t targetDistance, bool ahead) {
  // screenPrint("leaving");
  ahead ? setahead() : setback();
  resetTics();
  while (checkEncoderHealth() && getCurrentDistanceCm() < targetDistance) {
    if (buttonPressed)
      break;
    pidEncoders((kMinSpeedForward + kMaxSpeedForward) / 2, ahead);
  }
  stop();
}

float motors::getCurrentDistanceCm() {
  return getAverageTicks() * Geometry::kCmPerRisingTick;
}

float motors::getAngleOrientation() {
  bno.tryReadHeading(estimatedHeading_);
  float currentAngle = fmodf(estimatedHeading_, 360.0f);
  if (currentAngle < 0.0f) currentAngle += 360.0f;
  if ((currentAngle > 315 && currentAngle <= 360) ||
      (currentAngle >= 0 && currentAngle <= 45))
    return 0;
  else if (currentAngle > 45 && currentAngle <= 135)
    return 90;
  else if (currentAngle > 135 && currentAngle <= 225)
    return 180;
  else if (currentAngle > 225 && currentAngle <= 315)
    return 270;
  else
    return currentAngle;
}

void motors::resetOrientation() {
  bno.resetOrientation();
  estimatedHeading_ = 0.0f;
  targetAngle = 0;
}
void motors::checkpointElection() {
  float angleOrientation = getAngleOrientation();
  Serial.println("angle");
  String print;
  print = static_cast<String>(angleOrientation);
  // screenPrint(print);
  Serial.println(angleOrientation);
  constexpr float angleThreshold = Motion::kCheckpointAngleToleranceDegrees;
  const float currentAngle = angleOrientation + remainderf(estimatedHeading_ - angleOrientation, 360.0f);
  if (abs(currentAngle - angleOrientation) < angleThreshold) {
    return;
  }
  if (!rotate(angleOrientation)) return;
  if ((currentAngle - angleOrientation) < -angleThreshold) {
    ahead();
    if (left()) ahead();
  } else if ((currentAngle - angleOrientation) > angleThreshold) {
    ahead();
    if (right()) ahead();
  }
  return;
}

void motors::victimSequence() {
  const uint32_t startedMs = millis();
  while (millis() - startedMs < KitConfig::kVictimSequenceMs) {
    screenPrint("Victim");
    delay(KitConfig::kVictimBlinkMs);
    screenPrint(" ");
    delay(KitConfig::kVictimBlinkMs);
  }
}

void motors::harmedVictim() {
  victimSequence();
  if (kitState == kitID::kRight) {
    // screenPrint("Right");
    kitRight(2);
  } else if (kitState == kitID::kLeft) {
    // screenPrint("Left");
    kitLeft(2);
  }
  // screenPrint("");
}

void motors::stableVictim() {
  // screenPrint("Stable");
  victimSequence();
  if (kitState == kitID::kRight) {
    kitRight(1);
  } else if (kitState == kitID::kLeft) {
    kitLeft(1);
  }
  // screenPrint("");
}

void motors::unharmedVictim() {
  // screenPrint("Unharmed");
  victimSequence();
  // screenPrint("");
}

void motors::kitLeft(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    writeServo(servoPosRight);
    writeServo(servoPosLeft);
    writeServo(KitConfig::kLeftReleaseDegrees);
    writeServo(servoPosLeft);
    writeServo(KitConfig::kCenterDegrees);
  }
}
void motors::writeServo(uint16_t servoAngle) {
  servo.write(servoAngle);
  delay(KitConfig::kServoSettleMs);
  if ((servoAngle != KitConfig::kLeftReleaseDegrees && servoAngle != KitConfig::kCenterDegrees) && servoAngle != KitConfig::kRightReleaseDegrees) {
    for (uint8_t i = 0; i < 4; i++) {
      servo.write(servoAngle - KitConfig::kWiggleDegrees);
      delay(KitConfig::kWiggleStepMs);
      servo.write(servoAngle + KitConfig::kWiggleDegrees);
      delay(KitConfig::kWiggleStepMs);
    }
  }
}
void motors::kitRight(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    writeServo(servoPosLeft);
    writeServo(servoPosRight);
    writeServo(KitConfig::kRightReleaseDegrees);
    writeServo(servoPosRight);
    writeServo(KitConfig::kCenterDegrees);
  }
}
void motors::reloadKits() {
  writeServo(servoPosLeft);
  writeServo(servoPosRight);
  writeServo(KitConfig::kCenterDegrees);
}

void motors::wait(unsigned long targetTime) {
  unsigned long initialTime = millis();
  while ((millis() - initialTime) < targetTime) {
    if (buttonPressed)
      break;
  }
}
void motors::wifiPrint(String, float) {
  // client.print(message);
  // client.println(i);
  // Serial.println("Enviado: ");
}

void motors::setupTCS() {
  floorColorReady_ = MazeConfig::kUseFloorColor && tcs_.init();
  bno.resetOrientation();
}

void motors::screenBegin() {
  display.begin(SSD1306_SWITCHCAPVCC, Comms::kOledAddress, true, false);
  Serial.println("Screen initialized");
}
void motors::screenPrint(String output) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println(output);
  display.display();
}
