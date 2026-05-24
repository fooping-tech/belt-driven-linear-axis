#include "MotionController.h"

#include <math.h>

MotionController::MotionController(Axis& axis, float speedMmS) : axis_(axis), speedMmS_(speedMmS) {}

bool MotionController::moveToMm(float targetMm) {
  if (!axis_.isHomed() || !axis_.isWithinSoftLimit(targetMm)) {
    state_ = State::Error;
    return false;
  }

  targetSteps_ = lroundf(targetMm * axis_.stepsPerMm());
  const long delta = targetSteps_ - axis_.currentPositionSteps();
  if (delta == 0) {
    state_ = State::Idle;
    return true;
  }

  direction_ = delta > 0 ? 1 : -1;
  lastStepUs_ = micros();
  state_ = State::Moving;
  return true;
}

bool MotionController::moveToMm(float targetMm, float speedMmS) {
  speedMmS_ = speedMmS;
  return moveToMm(targetMm);
}

bool MotionController::moveRelativeMm(float deltaMm) {
  return moveToMm(axis_.currentPositionMm() + deltaMm);
}

bool MotionController::moveRelativeMm(float deltaMm, float speedMmS) {
  speedMmS_ = speedMmS;
  return moveRelativeMm(deltaMm);
}

void MotionController::update() {
  if (state_ != State::Moving) {
    return;
  }

  if (axis_.currentPositionSteps() == targetSteps_) {
    state_ = State::Idle;
    return;
  }

  if (!stepDue()) {
    return;
  }

  if (!axis_.moveOneStep(direction_)) {
    state_ = State::Error;
    return;
  }

  if (axis_.currentPositionSteps() == targetSteps_) {
    state_ = State::Idle;
  }
}

void MotionController::stop() {
  state_ = State::Idle;
}

bool MotionController::isMoving() const {
  return state_ == State::Moving;
}

bool MotionController::hasError() const {
  return state_ == State::Error;
}

MotionController::State MotionController::state() const {
  return state_;
}

const char* MotionController::stateName() const {
  switch (state_) {
    case State::Idle:
      return "Idle";
    case State::Moving:
      return "Moving";
    case State::Error:
      return "Error";
  }
  return "Unknown";
}

float MotionController::targetMm() const {
  return static_cast<float>(targetSteps_) / axis_.stepsPerMm();
}

float MotionController::speedMmS() const {
  return speedMmS_;
}

bool MotionController::stepDue() {
  const float stepsPerSecond = speedMmS_ * axis_.stepsPerMm();
  if (stepsPerSecond <= 0.0F) {
    state_ = State::Error;
    return false;
  }

  const uint32_t intervalUs = static_cast<uint32_t>(1000000.0F / stepsPerSecond);
  const uint32_t nowUs = micros();
  if (nowUs - lastStepUs_ < intervalUs) {
    return false;
  }

  lastStepUs_ = nowUs;
  return true;
}
