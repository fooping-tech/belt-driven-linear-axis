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
  void startHoming();
  void startMoveRelative(float mm);
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
  bool wasPressed_ = false;
  uint32_t pressStartedMs_ = 0;
  uint32_t lastDisplayMs_ = 0;
  HomingController::State lastHomingLogState_ = HomingController::State::Idle;
};
