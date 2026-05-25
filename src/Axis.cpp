#include "Axis.h"

#include <math.h>

Axis::Axis(StepDirDriver& driver, LimitSwitch* minLimit, float stepsPerMm, int minLimitDirection)
    : driver_(driver),
      minLimit_(minLimit),
      stepsPerMm_(stepsPerMm),
      minLimitDirection_(minLimitDirection < 0 ? -1 : 1) {}

void Axis::begin() {
  driver_.enable(true);
}

float Axis::currentPositionMm() const {
  return static_cast<float>(currentPositionSteps_) / stepsPerMm_;
}

long Axis::currentPositionSteps() const {
  return currentPositionSteps_;
}

float Axis::stepsPerMm() const {
  return stepsPerMm_;
}

void Axis::setStepsPerMm(float stepsPerMm) {
  stepsPerMm_ = stepsPerMm;
}

bool Axis::isHomed() const {
  return homed_;
}

void Axis::setHomed(bool homed) {
  homed_ = homed;
}

void Axis::setCurrentPositionSteps(long steps) {
  currentPositionSteps_ = steps;
}

void Axis::setCurrentPositionMm(float mm) {
  currentPositionSteps_ = lroundf(mm * stepsPerMm_);
}

void Axis::setSoftLimits(float minMm, float maxMm) {
  minMm_ = minMm;
  maxMm_ = maxMm;
  softLimitsEnabled_ = true;
}

bool Axis::isWithinSoftLimit(float targetMm) const {
  if (!softLimitsEnabled_) {
    return true;
  }
  return targetMm >= minMm_ && targetMm <= maxMm_;
}

bool Axis::isLimitPressed() {
  return minLimit_ != nullptr && minLimit_->isPressedDebounced();
}

bool Axis::moveOneStep(int direction) {
  if (!canMoveOneStep(direction, true, false)) {
    return false;
  }
  applyStep(direction);
  return true;
}

bool Axis::moveSteps(long steps, int direction) {
  const long count = labs(steps);
  for (long i = 0; i < count; ++i) {
    if (!moveOneStep(direction)) {
      return false;
    }
  }
  return true;
}

bool Axis::moveRelativeMm(float mm) {
  const int direction = mm >= 0.0F ? 1 : -1;
  return moveSteps(lroundf(fabsf(mm) * stepsPerMm_), direction);
}

bool Axis::moveOneStepForHoming(int direction) {
  if (!canMoveOneStep(direction, false, true)) {
    return false;
  }
  applyStep(direction);
  return true;
}

bool Axis::canMoveOneStep(int direction, bool requireHomed, bool ignoreLimit) {
  const int normalizedDirection = direction >= 0 ? 1 : -1;
  if (requireHomed && !homed_) {
    return false;
  }

  if (!ignoreLimit && normalizedDirection == minLimitDirection_ && isLimitPressed()) {
    return false;
  }

  if (requireHomed && softLimitsEnabled_) {
    const float nextMm = static_cast<float>(currentPositionSteps_ + normalizedDirection) / stepsPerMm_;
    if (!isWithinSoftLimit(nextMm)) {
      return false;
    }
  }

  return true;
}

void Axis::applyStep(int direction) {
  const int normalizedDirection = direction >= 0 ? 1 : -1;
  driver_.setDirection(normalizedDirection > 0);
  driver_.stepPulse();
  currentPositionSteps_ += normalizedDirection;
}
