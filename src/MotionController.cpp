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
  lastSpeedUpdateUs_ = lastStepUs_;
  currentSpeedStepsS_ = 0.0F;
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

  const uint32_t nowUs = micros();
  if (timingLastUpdateUs_ != 0) {
    const uint32_t gapUs = nowUs - timingLastUpdateUs_;
    timingSumUpdateGapUs_ += gapUs;
    if (gapUs > timingMaxUpdateGapUs_) {
      timingMaxUpdateGapUs_ = gapUs;
    }
  }
  timingLastUpdateUs_ = nowUs;
  ++timingUpdateCount_;

  if (axis_.currentPositionSteps() == targetSteps_) {
    state_ = State::Idle;
    return;
  }

  updateProfileSpeed();
  if (!stepDue(currentSpeedStepsS_)) {
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

void MotionController::resetTimingStats() {
  timingUpdateCount_ = 0;
  timingMaxUpdateGapUs_ = 0;
  timingSumUpdateGapUs_ = 0;
  timingLastUpdateUs_ = 0;
}

uint32_t MotionController::timingUpdateCount() const {
  return timingUpdateCount_;
}

uint32_t MotionController::timingMaxUpdateGapUs() const {
  return timingMaxUpdateGapUs_;
}

uint64_t MotionController::timingSumUpdateGapUs() const {
  return timingSumUpdateGapUs_;
}

float MotionController::timingAvgUpdateGapUs() const {
  const uint32_t gapCount = timingUpdateCount_ > 0 ? timingUpdateCount_ - 1 : 0;
  if (gapCount == 0) {
    return NAN;
  }
  return static_cast<float>(static_cast<double>(timingSumUpdateGapUs_) / static_cast<double>(gapCount));
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

long MotionController::targetSteps() const {
  return targetSteps_;
}

long MotionController::remainingSteps() const {
  return labs(targetSteps_ - axis_.currentPositionSteps());
}

float MotionController::speedMmS() const {
  return speedMmS_;
}

void MotionController::setAccelerationMmS2(float accelerationMmS2) {
  accelerationMmS2_ = accelerationMmS2;
}

float MotionController::accelerationMmS2() const {
  return accelerationMmS2_;
}

void MotionController::setProfile(Profile profile) {
  profile_ = profile;
}

MotionController::Profile MotionController::profile() const {
  return profile_;
}

const char* MotionController::profileName() const {
  switch (profile_) {
    case Profile::Direct:
      return "direct";
    case Profile::Trapezoid:
      return "trap";
  }
  return "unknown";
}

void MotionController::updateProfileSpeed() {
  const float targetStepsPerSecond = speedMmS_ * axis_.stepsPerMm();
  if (profile_ == Profile::Direct) {
    currentSpeedStepsS_ = targetStepsPerSecond;
    return;
  }

  const float accelerationStepsS2 = accelerationMmS2_ * axis_.stepsPerMm();
  if (targetStepsPerSecond <= 0.0F || accelerationStepsS2 <= 0.0F) {
    state_ = State::Error;
    return;
  }

  const uint32_t nowUs = micros();
  const float dtS = static_cast<float>(nowUs - lastSpeedUpdateUs_) / 1000000.0F;
  lastSpeedUpdateUs_ = nowUs;

  const long remainingSteps = labs(targetSteps_ - axis_.currentPositionSteps());
  const float stoppingSteps = (currentSpeedStepsS_ * currentSpeedStepsS_) / (2.0F * accelerationStepsS2);
  const bool shouldDecelerate = static_cast<float>(remainingSteps) <= stoppingSteps + 1.0F;
  const float deltaSpeed = accelerationStepsS2 * dtS;

  if (shouldDecelerate) {
    currentSpeedStepsS_ -= deltaSpeed;
  } else {
    currentSpeedStepsS_ += deltaSpeed;
  }

  const float minStepsPerSecond = fminf(targetStepsPerSecond, fmaxf(1.0F, accelerationStepsS2 * 0.005F));
  if (currentSpeedStepsS_ < minStepsPerSecond) {
    currentSpeedStepsS_ = minStepsPerSecond;
  }
  if (currentSpeedStepsS_ > targetStepsPerSecond) {
    currentSpeedStepsS_ = targetStepsPerSecond;
  }
}

bool MotionController::stepDue(float stepsPerSecond) {
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
