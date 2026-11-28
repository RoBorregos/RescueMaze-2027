#pragma once
#include "BNO.h"
#include "EncoderHealth.h"
#include "PID.h"
#include "config.h"
#include "TCS.h"
#include "VLX.h"
#include "motor.h"
#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define USING_SCREEN 1


constexpr float kTileLength = Geometry::kTileSizeCm;
constexpr uint8_t rulet[4][4] = {
    {0, 1, 2, 3}, {3, 0, 1, 2}, {2, 3, 0, 1}, {1, 2, 3, 0}};
constexpr float targetDistances[] = {MazeConfig::kCenterFrontMm / 10.0f,
                                     kTileLength + MazeConfig::kCenterFrontMm / 10.0f};
constexpr float targetDistancesB[] = {MazeConfig::kCenterBackMm / 10.0f,
                                      kTileLength + MazeConfig::kCenterBackMm / 10.0f};

namespace kitID {
constexpr uint8_t kRight = 2;
constexpr uint8_t kLeft = 1;
constexpr uint8_t kNone = 0;
} // namespace kitID
namespace rampID {
constexpr uint8_t kUp = 1;
constexpr uint8_t kDown = 2;
constexpr uint8_t kNone = 0;
} // namespace rampID

enum class TurnFeedback { kAutomatic, kBnoOnly, kEncodersOnly };

class motors {
private:
  float targetAngle = 0;
  static constexpr unsigned long delayTime = 350;
  PID myPID[4];
  EncoderHealth encoderHealth_;
  bool checkEncoderHealth();
  float estimatedHeading_ = 0.0f;
  float encoderTurnDegrees(const int *startTicks);
  bool floorColorReady_ = false;
  uint32_t lastFloorColorSampleMs_ = 0;
  // vlx
  static constexpr uint8_t kNumVlx = 8;
  static constexpr float maxVlxDistance = MazeConfig::kWallReferenceMaxMm / 10.0f;
  static constexpr uint8_t brakingDis = 5;
  static constexpr uint16_t kObstacleDistanceMm = kTileLength * 10 + 20;
  // wheels
  static constexpr float kTicsPerTile = kTileLength / Geometry::kCmPerRisingTick;
  // Speeds constants
  static constexpr float kMinSpeedForward = Control::toWheelPidTicks(Motion::kMinSpeedCmS);
  static constexpr float kMaxSpeedForward = Control::toWheelPidTicks(MazeConfig::kTravelSpeedCmS);
  static constexpr uint16_t kSpeedRampUp = 20;
  static constexpr uint16_t kSpeedRampDown = 9;
  // ramp
  PID rampUpPID;
  PID rampDownPID;
  bool slope = false;
  static constexpr float kMinRampOrientation = MazeConfig::kRampAngleDegrees;
  // control Walls
  float changeAngle = 0;
  static constexpr uint8_t maxChangeAngle = 3;
  // PID movement constants
  static constexpr float kP_mov = Control::kWheelKp;
  static constexpr float kI_mov = Control::kWheelKi;
  static constexpr float kD_mov = Control::kWheelKd;
  static constexpr uint32_t movTime = Control::kWheelPidPeriodMs;
  // ramp down constants
  static constexpr float kP_RampDown = 0.15;
  static constexpr float kI_RampDown = 0.00;
  static constexpr float kD_RampDown = 0.03;
  static constexpr uint8_t rampTime = 20;
  // ramp Up constants
  static constexpr float kP_RampUp = 0.2;
  static constexpr float kI_RampUp = 0.01;
  static constexpr float kD_RampUp = 0.1;
  // TCS
  char tileColor;
  static constexpr char kBlueColor = 'B';
  static constexpr char kBlackColor = 'N';
  static constexpr char kRedColor = 'R';
  static constexpr char kCheckpointColor = 'C';
  // movement
  bool limitColition = false;
  bool rampRealignmentPending_ = false;
  // servo
  static constexpr uint16_t servoPosRight = 133;
  static constexpr uint16_t servoPosLeft = 50;

public:
  // objets
  Adafruit_VL53L0X lox = Adafruit_VL53L0X();
  BNO bno;
  TCS tcs_;
  // LimitSwitch limitSwitch_[2];
  VLX vlx[kNumVlx];
  Servo servo;
  Motor motor[4];
  // Leds leds;
  // public variables
  bool sameOrientation = false;
  unsigned long buttonTime = millis();
  bool inMotion = false;
  bool blackTile = false;
  bool blueTile = false;
  bool checkpoint = false;
  uint8_t victim = 0;
  uint8_t kitState = kitID::kNone;
  bool buttonPressed = false;
  uint8_t rampState = rampID::kNone;
  motors(); // constructor
  // Motor speed control
  void setupMotors(bool initializeMazeSensors = true);
  void PID_Wheel(float, int);
  void PID_AllWheels(float);
  void pidEncoders(float, bool);
  void setSpeed(uint16_t);
  float changeSpeedMove(bool, bool, int, bool);
  // set movement
  void setahead();
  void setback();
  void setleft();
  void setright();
  void setleftTraslation();
  void setrightTraslation();
  void stop();
  void victimSequence();
  // movements
  void ahead();
  void back();
  bool left();
  bool right();
  bool rotate(float headingDegrees, TurnFeedback feedback = TurnFeedback::kAutomatic);
  bool rotateByEncoders(float degrees);
  void moveDistance(uint8_t targetDistance, bool);
  void writeServo(uint16_t servoAngle);
  // setups
  void setupTCS();
  void setupVlx(const uint8_t);
  void resetOrientation();
  void resetVlx();
  // sensors
  void checkTileColor(bool atEnd = false);
  void limitCrash();
  float nearWall();
  bool passObstacle();
  WallReading readWall(uint8_t direction);
  // victims
  void harmedVictim();
  void stableVictim();
  void unharmedVictim();
  void kitRight(uint8_t);
  void kitLeft(uint8_t);
  void reloadKits();
  // gets
  float getCurrentDistanceCm();
  float getAngleOrientation();
  double getAverageTicks();
  double getTicsSpeed();
  // logic
  void checkpointElection();
  float findNearest(float, const float[], uint8_t, bool);
  float calculateAngularDistance();
  void resetTics();
  bool hasEncoderFeedback(uint8_t wheel) const {
    return wheel < MotorID::kCount && !encoderHealth_.failed(wheel);
  }
  // ramp
  bool rampInFront();
  bool isRamp();
  void ramp();
  bool isRampRealignmentPending() const { return rampRealignmentPending_; }
  // comunication
  void wait(unsigned long);
  void wifiPrint(String, float);
  void screenBegin();
  void screenPrint(String);
  void printSpeeds();
  void printAngle();
};
