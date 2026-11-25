#include "motors.h"
#include "TileAdvance.h"
#include "Encoder.h"
#include "Pins_ID.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
motors::motors() {}

void motors::setupMotors() {
  Wire.setSDA(Pins::i2cSda);
  Wire.setSCL(Pins::i2cScl);
  Wire.begin();
  Wire1.setSDA(Pins::i2c1Sda);
  Wire1.setSCL(Pins::i2c1Scl);
  Wire1.begin();
  Wire.setClock(100000);
  Wire1.setClock(100000);
  bno.setupBNO();
  resetVlx();
  setupTCS();

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

void motors::PID_speed(float setpoint, float angle, uint16_t reference_speed) {
  checkEncoderHealth();
  PID PID;
  double output = PID.calculate_PID(setpoint, angle);
  int right_speed = reference_speed - output;
  int left_speed = reference_speed + output;
  right_speed = constrain(right_speed, 30, 255);
  left_speed = constrain(left_speed, 30, 255);
  for (int i = 0; i < 4; i++) {
    motor[i].setSpeed((i % 2 == 0) ? left_speed : right_speed);
  }
  delay(20);
}
void motors::checkEncoderHealth() {
  const uint32_t now = millis();
  for (uint8_t wheel = 0; wheel < MotorID::kCount; ++wheel) {
    if (encoderHealth_.observe(wheel, motor[wheel].getTotalEncoderTics(),
                               motor[wheel].getSpeed(), now)) {
      Serial.print("Encoder signal lost on wheel slot ");
      Serial.println(wheel);
    }
  }
  if (!encoderHealth_.hasFeedbackOnBothSides()) {
    stop();
    Serial.println("Stopped: both encoder signals lost on one side.");
    while (true) delay(50);
  }
}

void motors::PID_Wheel(float targetSpeed, int i) {
  checkEncoderHealth();
  if (encoderHealth_.failed(i)) return;
  const float speed_setpoint = targetSpeed;
  int reference_pwm = motor[i].getSpeed();
  Serial.println("reference_pwm");
  Serial.println(reference_pwm);
  const float speedTics = fabsf(motor[i].getTicsSpeed()); // The 2026 PID controls speed magnitude.
  Serial.println("speedTics");
  Serial.println(speedTics);
  float error = myPID[i].calculate_PID(speed_setpoint, speedTics);
  int speed = reference_pwm + error;
  speed = constrain(speed, 0, static_cast<int>(Control::kPwmMax));
  motor[i].setSpeed(speed);
  const uint8_t backupWheel = EncoderHealth::partner(i);
  if (encoderHealth_.failed(backupWheel)) motor[backupWheel].setSpeed(speed);
}
void motors::PID_AllWheels(float targetSpeed) {
  for (uint8_t i = 0; i < 4; i++) {
    PID_Wheel(targetSpeed, i);
  }
}

void motors::pidEncoders(float speedReference, bool ahead) {
  bno.getOrientationX();
  static PID pidBno(0.5, 0.1, 0.01, 1);
  if (rampState != 0)
    changeAngle = 0;
  float AngleError = pidBno.calculate_PID(
      targetAngle + changeAngle, (targetAngle == 0 ? z_rotation : angle));
  AngleError = constrain(AngleError, -17, 17);
  // Serial.println(AngleError);
  Serial.println(AngleError);
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
  passObstacle();
  passObstacle(); // Double check before starting the tile.
  nearWall();
  resetTics();

  using Reference = TileAdvance::Reference;
  const auto readReference = [this](uint8_t slot, uint16_t maxMm, float &cm) {
    uint16_t mm = 0;
    if (!vlx[slot].readValidMillimeters(mm) || mm < 10 || mm >= maxMm) return false;
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

  const bool startsOnSlope = abs(bno.getOrientationY()) > 15;
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
  while (!advance.done(remaining)) {
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
      validWall = sampleWall(sampleCm) || sampleWall(sampleCm);
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
      const bool haveLeft = readReference(vlxID::frontLeft, maxFrontMm, leftCm);
      const bool haveRight = readReference(vlxID::frontRight, maxFrontMm, rightCm);
      const float encoderCm = getCurrentDistanceCm();
      remaining = advance.remainingCm(encoderCm, true, wallCm);
      if (haveLeft || haveRight) {
        const bool pickLeft = haveLeft && (!haveRight || leftCm >= rightCm);
        const uint8_t candidateSlot = pickLeft ? vlxID::frontLeft : vlxID::frontRight;
        const float candidateCm = pickLeft ? leftCm : rightCm;
        if (!advance.done(remaining) && candidate.observe(candidateSlot, candidateCm, encoderCm) &&
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
  if (!blackTile && !buttonPressed) checkTileColor(true);
  resetTics();
}

void motors::checkTileColor(bool atEnd) {
  if (!floorColorReady_ || buttonPressed) return;
  const uint32_t now = millis();
  if (!atEnd && now - lastFloorColorSampleMs_ < 50) return;
  lastFloorColorSampleMs_ = now;
  const char tileColor = tcs_.getColor();
  if (tileColor == kBlackColor) {
    stop();
    const long returnTics = getAvergeTics();
    blackTile = true;
    Serial.println("Black Tile");
    resetTics();
    const uint32_t retreatStart = millis();
    while (getAvergeTics() < returnTics && !buttonPressed &&
           millis() - retreatStart < 3000) {
      setback();
      const float remainingCm = (returnTics - getAvergeTics()) * Geometry::kCmPerRisingTick;
      pidEncoders(advancePidTarget(remainingCm, false), false);
    }
    stop();
    if (!buttonPressed && getAvergeTics() < returnTics) {
      Serial.println("Stopped: could not return from black tile.");
      while (true) delay(50);
    }
    resetTics();
  } else if (tileColor == kBlueColor && atEnd) {
    // screenPrint("Blue Tile");
    Serial.println("Blue Tile");
    blueTile = true;
    wait(5500);
  } else if (tileColor == kCheckpointColor && atEnd) {
    checkpoint = true;
    Serial.println("Checkpoint Tile");
    // screenPrint("Checkpoint Tile");
  }
}

float motors::nearWall() {
  // float left = AverageLeftDistance();
  // float right = AverageRightDistance();
  if (vlx[vlxID::leftFront].getDistance() < minDisToLateralWall) {
    changeAngle = maxChangeAngle;
  } else if (vlx[vlxID::rightFront].getDistance() < minDisToLateralWall) {
    changeAngle = -maxChangeAngle;
  } else {
    changeAngle = 0;
  }
  return changeAngle;
}

void motors::passObstacle() {
  float targetAngle_ = targetAngle;
  float sideAngle = targetAngle_;
  float angle = bno.getOrientationX();
  if (abs(targetAngle_ - angle) >= 2)
    rotate(targetAngle_);
  bool leftBlocked = vlx[vlxID::frontLeft].getDistance() < kDistanceToObstacle;
  bool rightBlocked =
      vlx[vlxID::frontRight].getDistance() < kDistanceToObstacle;

  if (!leftBlocked && !rightBlocked)
    return; // No obstacle, do nothing
  if (leftBlocked && rightBlocked)
    return; // Completely blocked, can't pass

  moveDistance(kTileLength / 5, false);
  limitColition = true;
  if (leftBlocked || rightBlocked) {
    float sideAngle = targetAngle + (leftBlocked ? 25 : -25);
    if (sideAngle >= 360)
      sideAngle -= 360;
    if (sideAngle < 0)
      sideAngle += 360;

    rotate(sideAngle);
    moveDistance(3 * kTileLength / 10, true);
  }
  rotate(targetAngle_);
  limitColition = false;
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
void motors::right() {
  Serial.println("right");
  targetAngle = targetAngle + 90;
  rotate(targetAngle);
  if (targetAngle == 360) {
    targetAngle = 0;
  }
}
void motors::left() {
  Serial.println("left");
  if (targetAngle == 0) {
    targetAngle = 360;
  }
  targetAngle = targetAngle - 90;
  rotate(targetAngle);
}

float motors::calculateAngularDistance() {
  float rightAngularDistance, leftAngularDistance;
  if (targetAngle >= angle) {
    rightAngularDistance = targetAngle - angle;
    leftAngularDistance = angle + (360 - targetAngle);
  } else {
    rightAngularDistance = (360 - angle) + targetAngle;
    leftAngularDistance = angle - targetAngle;
  }
  return (rightAngularDistance <= leftAngularDistance) ? rightAngularDistance
                                                       : -leftAngularDistance;
}
void motors::rotate(float deltaAngle) {
  String print = "Turn " + static_cast<String>(deltaAngle);
  // screenPrint(print);
  targetAngle = deltaAngle;
  delayMicroseconds(1);
  bno.getOrientationX();
  float currentAngle, rightAngularDistance, leftAngularDistance, minInterval,
      maxInterval, tolerance = Motion::kTurnToleranceDegrees;
  bool hexadecimal;
  // calculate angular distance in both directions
  if (targetAngle >= angle) {
    rightAngularDistance = targetAngle - angle;
    leftAngularDistance = angle + (360 - targetAngle);
  } else {
    rightAngularDistance = (360 - angle) + targetAngle;
    leftAngularDistance = angle - targetAngle;
  }
  // define target interval and use angle or z_rotation
  if (targetAngle != 360 && targetAngle != 0) {
    minInterval = (targetAngle - tolerance),
    maxInterval = (targetAngle + tolerance);
    hexadecimal = true;
  } else {
    targetAngle = 0;
    minInterval = -tolerance, maxInterval = tolerance;
    hexadecimal = false;
  }
  // decide shortest route and rotate
  (rightAngularDistance <= leftAngularDistance) ? (setright()) : (setleft());

  currentAngle = hexadecimal ? angle : z_rotation;
  while (currentAngle < minInterval || currentAngle > maxInterval) {
    changeSpeedMove(false, true, 0, false);
    bno.getOrientationX();
    currentAngle = hexadecimal ? angle : z_rotation;
    if (buttonPressed)
      break;
    // Serial.println(angle);
  }
  stop();
  inMotion = true;
}

float motors::changeSpeedMove(bool encoders, bool rotate, int targetDistance,
                              bool frontVlx) {
  float speed;
  float missingDistance, missingAngle;
  if (rotate == true) {
    missingAngle = fabsf(remainderf(targetAngle - angle, 360.0f));
    speed = Control::toWheelPidTicks(constrain(0.24f * missingAngle, 2.5f, Motion::kTurnSpeedCmS));
    PID_AllWheels(speed);
    return 0;
  } else {
    if (encoders == true) {
      speed =
          map(getAvergeTics(), 0, kTicsPerTile, kMaxPwmFormard, kMinPwmFormard);
      missingDistance =
          kTileLength - (getAvergeTics() * kTileLength / kTicsPerTile);
      return speed;
    } else {
      missingDistance = abs((frontVlx ? vlx[vlxID::frontLeft].getDistance()
                                      : vlx[vlxID::backLeft].getDistance()) -
                            targetDistance); // intercambiar vlx
      speed = frontVlx ? map(missingDistance, kTileLength, 0, kMaxPwmFormard,
                             kMinPwmFormard)
                       : map(missingDistance, kTileLength, 0, kMaxPwmFormard,
                             kMinPwmFormard);
      speed = constrain(speed, kMinPwmFormard, kMaxPwmFormard);
      return speed;
    }
  }
}

void motors::setSpeed(uint16_t speed) {
  if (speed != 0) checkEncoderHealth();
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
double motors::getAvergeTics() {
  checkEncoderHealth();
  int counts[MotorID::kCount];
  for (uint8_t wheel = 0; wheel < MotorID::kCount; ++wheel)
    counts[wheel] = abs(motor[wheel].getTics()); // The 2026 moves compare distance magnitude.
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
  Serial.println(vlxID::connectors[index]);
  while (true) delay(50);
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
  const bool firstInstalled = vlxID::connectors[slots[0]] != vlxID::kNone;
  const bool secondInstalled = vlxID::connectors[slots[1]] != vlxID::kNone;

  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    const WallReading first = firstInstalled ? vlx[slots[0]].readWall(wallDistance)
                                             : WallReading::kUnknown;
    const WallReading second = secondInstalled ? vlx[slots[1]].readWall(wallDistance)
                                               : WallReading::kUnknown;
    if (firstInstalled && secondInstalled) {
      if (first != WallReading::kUnknown && first == second) return first;
    } else if (first != WallReading::kUnknown || second != WallReading::kUnknown) {
      return first != WallReading::kUnknown ? first : second;
    }
  }
  return WallReading::kUnknown;
}

bool motors::rampInFront() {
  if ((vlx[vlxID::frontLeft].getDistance() -
       vlx[vlxID::frontRight].getDistance()) >= 2) {
    return true;
  } else {
    return false;
  }
}
bool motors::isRamp() {
  float currentOrientationY = bno.getOrientationY();
  // screenPrint("isRamp");
  if (abs(currentOrientationY) > 7)
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
  while (bno.getOrientationY() > 7) {
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
    if ((vlx[vlxID::rightFront].distance < vlx[vlxID::rightFront].kDistanceToWall &&
         vlx[vlxID::leftFront].distance < vlx[vlxID::leftFront].kDistanceToWall) &&
        (vlx[vlxID::rightFront].distance < 6 || vlx[vlxID::leftFront].distance < 6)) {
      error = rampUpPID.calculate_PID(
          0, (vlx[vlxID::rightFront].distance - vlx[vlxID::leftFront].distance));
      error = constrain(error, -15, 15);
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
  while (bno.getOrientationY() < -7) {
    if (buttonPressed == true)
      break;
    // limitCrash();
    if (limitColition == true)
      break;
    float error;
    vlx[vlxID::rightFront].getDistance();
    vlx[vlxID::leftFront].getDistance();
    if ((vlx[vlxID::rightFront].distance < vlx[vlxID::rightFront].kDistanceToWall &&
         vlx[vlxID::leftFront].distance < vlx[vlxID::leftFront].kDistanceToWall) &&
        (vlx[vlxID::rightFront].distance < 6 || vlx[vlxID::leftFront].distance < 6)) {
      error = rampDownPID.calculate_PID(
          0, (vlx[vlxID::rightFront].distance - vlx[vlxID::leftFront].distance));
      error = constrain(error, -6, 6);
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
  if (getAvergeTics() > 1 * kTicsPerTile && rampState == 1) {
    // stop();
    // bno.resetOrientationX();
    // setahead();
    moveDistance(kTileLength / 3, true);
    rotate(targetAngle);
  } else if (getAvergeTics() > 0.8 * kTicsPerTile && rampState == 2) {
    // stop();
    // bno.resetOrientationX();
    // setahead();
    moveDistance(kTileLength / 2, true);
    rotate(targetAngle);
  } else {
    rampState = 0;
  }
  limitColition = false;
  resetTics();
  stop();
  wait(100);
}

void motors::moveDistance(uint8_t targetDistance, bool ahead) {
  // screenPrint("leaving");
  ahead ? setahead() : setback();
  resetTics();
  while (getCurrentDistanceCm() < targetDistance) {
    if (buttonPressed)
      break;
    pidEncoders((kMinSpeedFormard + kMaxSpeedFormard) / 2, ahead);
  }
  stop();
}

float motors::getCurrentDistanceCm() {
  return getAvergeTics() * Geometry::kCmPerRisingTick;
}

float motors::getAngleOrientation() {
  float currentAngle = bno.getOrientationX();
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
  targetAngle = 0;
}
void motors::checkpointElection() {
  float angleOrientation = getAngleOrientation();
  Serial.println("angle");
  String print;
  print = static_cast<String>(angleOrientation);
  // screenPrint(print);
  Serial.println(angleOrientation);
  uint8_t angleThreshold = 10;
  float currentAngle = (angleOrientation == 0) ? z_rotation : angle;
  if (abs(currentAngle - angleOrientation) < angleThreshold) {
    return;
  }
  rotate(angleOrientation);
  int turn;
  if ((currentAngle - angleOrientation) < -angleThreshold) {
    turn = -1;
    ahead();
    left();
    ahead();
  } else if ((currentAngle - angleOrientation) > angleThreshold) {
    turn = 1;
    ahead();
    right();
    ahead();
  }
  return;
}

void motors::victimSequency() {
  float current = millis();
  while ((millis() - current) < 5100) {
    screenPrint("Victim");
    delay(500);
    screenPrint(" ");
    delay(500);
  }
}

void motors::harmedVictim() {
  victimSequency();
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
  victimSequency();
  if (kitState == kitID::kRight) {
    kitRight(1);
  } else if (kitState == kitID::kLeft) {
    kitLeft(1);
  }
  // screenPrint("");
}

void motors::unharmedVictim() {
  // screenPrint("Unharmed");
  victimSequency();
  // screenPrint("");
}

void motors::kitLeft(uint8_t n) {
  uint16_t dt = 0;
  for (uint8_t i = 0; i < n; i++) {
    writeServo(servoPosRight);
    writeServo(servoPosLeft);
    writeServo(10);
    writeServo(servoPosLeft);
    writeServo(90);
  }
}
void motors::writeServo(uint16_t servoAngle) {
  servo.write(servoAngle);
  delay(200);
  if ((servoAngle != 10 && servoAngle != 90) && servoAngle != 170) {
    for (uint8_t i; i < 4; i++) {
      servo.write(servoAngle - 6);
      delay(40);
      servo.write(servoAngle + 6);
      delay(40);
    }
  }
}
void motors::kitRight(uint8_t n) {
  uint16_t dt = 600;
  for (uint8_t i = 0; i < n; i++) {
    writeServo(servoPosLeft);
    writeServo(servoPosRight);
    writeServo(170);
    writeServo(servoPosRight);
    writeServo(90);
  }
}
void motors::reloadKits() {
  writeServo(servoPosLeft);
  writeServo(servoPosRight);
  writeServo(90);
}

void motors::wait(unsigned long targetTime) {
  unsigned long initialTime = millis();
  while ((millis() - initialTime) < targetTime) {
    if (buttonPressed)
      break;
  }
}
void motors::wifiPrint(String message, float i) {
  // client.print(message);
  // client.println(i);
  // Serial.println("Enviado: ");
}

void motors::setupTCS() {
  floorColorReady_ = MazeConfig::kUseFloorColor && tcs_.init();
  bno.resetOrientation();
}

void motors::screenBegin() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false);
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

void motors::calibrateColors() {
  uint16_t dt = 10000;
  // screenPrint("Blue Tile");
  Serial.println("Blue Tile");
  delay(dt);
  tcs_.updateRGBC();
  tcs_.printRGB();
  float redInBlue = tcs_.red_;
  float greenInBlue = tcs_.green_;
  float blueInBlue = tcs_.blue_;

  // screenPrint("Black Tile");
  Serial.println("Black Tile");
  delay(dt);
  tcs_.updateRGBC();
  tcs_.printRGB();
  float redInBlack = tcs_.red_;
  float greenInBlack = tcs_.green_;
  float blueInBlack = tcs_.blue_;

  // screenPrint("Checkpoint Tile");
  Serial.println("Checkpoint Tile");
  delay(dt);
  tcs_.updateRGBC();
  tcs_.printRGB();
  float redInCheck = tcs_.red_;
  float greenInCheck = tcs_.green_;
  float blueInCheck = tcs_.blue_;
  float clearInCheck = tcs_.clear_;

  // screenPrint("white Tile");
  Serial.println("White Tile");
  delay(dt);
  tcs_.updateRGBC();
  tcs_.printRGB();
  float redInWhite = tcs_.red_;
  float greenInWhite = tcs_.green_;
  float blueInWhite = tcs_.blue_;

  while (true) {
    if (buttonPressed)
      break;
    String print;
    dt = 2000;
    Serial.print("RGB Blue");
    delay(800);
    print = static_cast<String>(redInBlue) + "," +
            static_cast<String>(greenInBlue) + "," +
            static_cast<String>(blueInBlue);
    Serial.println(print);
    delay(dt);

    // screenPrint("RGB Black");
    Serial.print("RGB Black");
    delay(800);
    print = static_cast<String>(redInBlack) + "," +
            static_cast<String>(greenInBlack) + "," +
            static_cast<String>(blueInBlack);
    Serial.println(print);
    delay(dt);

    // screenPrint("RGB Check");
    Serial.print("RGB Checkpoint");
    delay(800);
    print = static_cast<String>(redInCheck) + "," +
            static_cast<String>(greenInCheck) + "," +
            static_cast<String>(blueInCheck);
    Serial.println(print);
    delay(dt);

    // screenPrint("RGB white");
    Serial.print("RGB White");
    delay(800);
    print = static_cast<String>(redInWhite) + "," +
            static_cast<String>(greenInWhite) + "," +
            static_cast<String>(blueInWhite);
    Serial.println(print);
    delay(dt);
  }
}
