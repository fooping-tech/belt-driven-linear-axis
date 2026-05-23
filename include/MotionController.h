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

  MotionController(Axis& axis, float speedMmS);

  bool moveToMm(float targetMm);
  bool moveRelativeMm(float deltaMm);
  void update();
  void stop();
  bool isMoving() const;
  bool hasError() const;
  State state() const;
  const char* stateName() const;
  float targetMm() const;

 private:
  bool stepDue();

  Axis& axis_;
  float speedMmS_;
  long targetSteps_ = 0;
  int direction_ = 1;
  State state_ = State::Idle;
  uint32_t lastStepUs_ = 0;
};

