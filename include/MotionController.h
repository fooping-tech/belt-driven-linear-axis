#pragma once

#include <Arduino.h>

#include "Axis.h"

class MotionController {
 public:
  enum class State : uint8_t {
    Idle,
    Moving,
    Error,
  };

  enum class Profile : uint8_t {
    Direct,
    Trapezoid,
  };

  MotionController(Axis& axis, float speedMmS);

  bool moveToMm(float targetMm);
  bool moveToMm(float targetMm, float speedMmS);
  bool moveRelativeMm(float deltaMm);
  bool moveRelativeMm(float deltaMm, float speedMmS);
  void update();
  void stop();
  void resetTimingStats();
  uint32_t timingUpdateCount() const;
  uint32_t timingMaxUpdateGapUs() const;
  uint64_t timingSumUpdateGapUs() const;
  float timingAvgUpdateGapUs() const;
  bool isMoving() const;
  bool hasError() const;
  State state() const;
  const char* stateName() const;
  float targetMm() const;
  long targetSteps() const;
  long remainingSteps() const;
  float speedMmS() const;
  void setAccelerationMmS2(float accelerationMmS2);
  float accelerationMmS2() const;
  void setProfile(Profile profile);
  Profile profile() const;
  const char* profileName() const;

 private:
  void updateProfileSpeed();
  bool stepDue(float stepsPerSecond);

  Axis& axis_;
  float speedMmS_;
  float accelerationMmS2_ = 100.0F;
  float currentSpeedStepsS_ = 0.0F;
  long targetSteps_ = 0;
  int direction_ = 1;
  Profile profile_ = Profile::Trapezoid;
  State state_ = State::Idle;
  uint32_t lastStepUs_ = 0;
  uint32_t lastSpeedUpdateUs_ = 0;
  uint32_t timingUpdateCount_ = 0;
  uint32_t timingMaxUpdateGapUs_ = 0;
  uint64_t timingSumUpdateGapUs_ = 0;
  uint32_t timingLastUpdateUs_ = 0;
};
