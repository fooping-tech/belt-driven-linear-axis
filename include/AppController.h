#pragma once

#include <Arduino.h>
#include <M5Unified.h>

#include "As5600Sensor.h"
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
  void setRuntimeCurrent(uint16_t currentMa);
  void setChopMode(bool spreadCycle);
  void setMicrosteps(uint16_t microsteps);
  bool readStallGuardResult(uint16_t& sgResult);
  void updateStallGuardStats();
  void resetStallGuardStats();
  void printStallGuardStatus();
  void printStallGuardSummary();
  void printMotionTimingSummary();
  void printMoveTiming(uint32_t targetReachedUs, uint32_t completePrintBeforeUs) const;
  void printRejectDetail(const char* stage, const char* reason);
  void printCurrentStatusCsv(const char* driverStatus);
  void initAs5600();
  void printAs5600Angle();
  void printAs5600BitBangStatus();
  void printI2cScan();
  void printI2cBitBangScan();
  void printAs5600Status();
  void printIoStatus();
  void runI2cPinPulseTest();
  void runMotorStepTest();
  void updateHeartbeatAndLoopStats();
  void printLoopDiagnostics() const;
  void printResetReason() const;
  const char* resetReasonName() const;
  void playStartupMotorMelody();
  bool validateTmcUartForMove(char* failReason, size_t failReasonSize);
  bool setStallGuardThreshold(uint16_t sgthrs);
  bool setStallGuardTcoolThreshold(uint32_t tcoolthrs);
  bool parseUnsignedLongCommand(const String& line, const char* prefix, uint32_t& value) const;
  void startHoming();
  void startMoveRelative(float mm);
  void startMoveRelative(float mm, float speedMmS);
  bool parseSpeedCommand(const String& line, float& speedMmS) const;
  bool parseAccelCommand(const String& line, float& accelerationMmS2) const;
  bool parseUnsignedCommand(const String& line, uint16_t& value) const;
  bool parseMoveCommand(const String& line, float& distanceMm, float& speedMmS) const;
  bool validateMoveSpeed(float speedMmS) const;
  bool validateAcceleration(float accelerationMmS2) const;
  bool validateMoveRequest(float distanceMm, float speedMmS);
  void printMoveUsage() const;
  void updateState();
  void printStatus();
  void printDiagnosticStatus();
  void drawStatus();
  const char* stateName() const;

  StepDirDriver driver_;
  LimitSwitch limit_;
  As5600Sensor as5600_;
  Axis axis_;
  HomingController homing_;
  MotionController motion_;
  State state_ = State::Boot;
  bool motorPowerEnabled_ = true;
  float defaultMoveSpeedMmS_ = DEFAULT_MOVE_SPEED_MM_S;
  bool wasPressed_ = false;
  uint32_t pressStartedMs_ = 0;
  uint32_t lastDisplayMs_ = 0;
  uint32_t lastLoopTickUs_ = 0;
  uint32_t lastLoopGapUs_ = 0;
  uint32_t maxLoopGapUs_ = 0;
  uint32_t lastHeartbeatToggleMs_ = 0;
  bool heartbeatState_ = false;
  int resetReason_ = 0;
  uint32_t moveStartedUs_ = 0;
  float moveCommandDeltaMm_ = 0.0F;
  float moveCommandSpeedMmS_ = 0.0F;
  float moveCommandAccelMmS2_ = 0.0F;
  long moveTargetSteps_ = 0;
  long moveStartSteps_ = 0;
  HomingController::State lastHomingLogState_ = HomingController::State::Idle;
  String serialLine_;
};
