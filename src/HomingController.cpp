#include "HomingController.h"

#include <math.h>

HomingController::HomingController(Axis& axis, const HomingConfig& config) : axis_(axis), config_(config) {}

void HomingController::start() {
  axis_.setHomed(false);
  travelSteps_ = 0;
  backoffSteps_ = 0;
  lastStepUs_ = micros();
  state_ = State::SeekFast;
}

void HomingController::update() {
  switch (state_) {
    case State::Idle:
    case State::Done:
    case State::Error:
      return;

    case State::SeekFast:
      if (axis_.isLimitPressed()) {
        state_ = State::Backoff;
        backoffSteps_ = 0;
        return;
      }
      if (travelSteps_ >= mmToSteps(config_.maxTravelMm)) {
        markError();
        return;
      }
      if (stepDue(config_.fastSpeedMmS) && axis_.moveOneStepForHoming(config_.direction)) {
        ++travelSteps_;
      }
      return;

    case State::Backoff:
      if (!axis_.isLimitPressed()) {
        travelSteps_ = 0;
        state_ = State::SeekSlow;
        return;
      }
      if (backoffSteps_ >= mmToSteps(config_.backoffMm)) {
        markError();
        return;
      }
      if (stepDue(config_.fastSpeedMmS) && axis_.moveOneStepForHoming(-config_.direction)) {
        ++backoffSteps_;
      }
      return;

    case State::SeekSlow:
      if (axis_.isLimitPressed()) {
        state_ = State::SetZero;
        return;
      }
      if (travelSteps_ >= mmToSteps(config_.maxTravelMm)) {
        markError();
        return;
      }
      if (stepDue(config_.slowSpeedMmS) && axis_.moveOneStepForHoming(config_.direction)) {
        ++travelSteps_;
      }
      return;

    case State::SetZero:
      axis_.setCurrentPositionMm(0.0F);
      axis_.setHomed(true);
      state_ = State::Done;
      return;
  }
}

bool HomingController::isBusy() const {
  return state_ == State::SeekFast || state_ == State::Backoff || state_ == State::SeekSlow ||
         state_ == State::SetZero;
}

bool HomingController::isDone() const {
  return state_ == State::Done;
}

bool HomingController::hasError() const {
  return state_ == State::Error;
}

void HomingController::reset() {
  state_ = State::Idle;
  travelSteps_ = 0;
  backoffSteps_ = 0;
}

HomingController::State HomingController::state() const {
  return state_;
}

const char* HomingController::stateName() const {
  switch (state_) {
    case State::Idle:
      return "Idle";
    case State::SeekFast:
      return "SeekFast";
    case State::Backoff:
      return "Backoff";
    case State::SeekSlow:
      return "SeekSlow";
    case State::SetZero:
      return "SetZero";
    case State::Done:
      return "Done";
    case State::Error:
      return "Error";
  }
  return "Unknown";
}

bool HomingController::stepDue(float speedMmS) {
  const float stepsPerSecond = speedMmS * axis_.stepsPerMm();
  if (stepsPerSecond <= 0.0F) {
    markError();
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

void HomingController::markError() {
  axis_.setHomed(false);
  state_ = State::Error;
}

long HomingController::mmToSteps(float mm) const {
  return lroundf(mm * axis_.stepsPerMm());
}
