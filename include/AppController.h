#pragma once

#include <Arduino.h>
#include <M5Unified.h>

#include "Axis.h"
#include "HomingController.h"
#include "LimitSwitch.h"
#include "MotionController.h"
#include "StepDirDriver.h"
#include "config.h"

class AppController {
 public:
  enum class State : uint8_t {
    Boot,
    NotHomed,
    Homing,
    Ready,
    Moving,
    Error,
  };

  AppController();

  void begin();
  void update();

 private:
  void handleButton();
  void handleSerial();
  void processSerialLine(const String& line);
  void initDriverUart();
  void setMotorPower(bool enabled);
  void startHoming();
  void startMoveRelative(float mm);
  void startMoveRelative(float mm, float speedMmS);
  bool parseMoveCommand(const String& line, float& distanceMm, float& speedMmS) const;
  bool validateMoveRequest(float distanceMm, float speedMmS) const;
  void printMoveUsage() const;
  void updateState();
  void printStatus();
  void drawStatus();
  const char* stateName() const;

  StepDirDriver driver_;
  LimitSwitch limit_;
  Axis axis_;
  HomingController homing_;
  MotionController motion_;
  State state_ = State::Boot;
  bool motorPowerEnabled_ = true;
  bool wasPressed_ = false;
  uint32_t pressStartedMs_ = 0;
  uint32_t lastDisplayMs_ = 0;
  HomingController::State lastHomingLogState_ = HomingController::State::Idle;
  String serialLine_;
};
