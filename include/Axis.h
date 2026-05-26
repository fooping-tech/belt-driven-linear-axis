#pragma once

#include <Arduino.h>

#include "LimitSwitch.h"
#include "StepDirDriver.h"

class Axis {
 public:
  Axis(StepDirDriver& driver, LimitSwitch* minLimit, float stepsPerMm, int minLimitDirection = -1);

  void begin();
  float currentPositionMm() const;
  long currentPositionSteps() const;
  float stepsPerMm() const;
  void setStepsPerMm(float stepsPerMm);
  bool isHomed() const;
  void setHomed(bool homed);
  void setCurrentPositionSteps(long steps);
  void setCurrentPositionMm(float mm);
  void setSoftLimits(float minMm, float maxMm);
  bool isWithinSoftLimit(float targetMm) const;
  bool isLimitPressed();
  bool moveOneStep(int direction);
  bool moveSteps(long steps, int direction);
  bool moveRelativeMm(float mm);
  bool moveOneStepForHoming(int direction);
  const char* lastMoveRejectReasonName() const;

 private:
  enum class MoveRejectReason : uint8_t {
    None,
    LimitPressed,
    SoftLimitMin,
    SoftLimitMax,
    DriverDisabled,
    Unknown,
  };

  bool canMoveOneStep(int direction, bool requireHomed, bool ignoreLimit);
  void applyStep(int direction);
  void setMoveRejectReason(MoveRejectReason reason);

  StepDirDriver& driver_;
  LimitSwitch* minLimit_;
  float stepsPerMm_;
  int minLimitDirection_;
  long currentPositionSteps_ = 0;
  float minMm_ = 0.0F;
  float maxMm_ = 0.0F;
  bool softLimitsEnabled_ = false;
  bool homed_ = false;
  MoveRejectReason lastMoveRejectReason_ = MoveRejectReason::None;
};
