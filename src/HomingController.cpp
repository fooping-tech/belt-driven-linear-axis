#include "HomingController.h"

#include <math.h>

HomingController::HomingController(Axis& axis, const HomingConfig& config) : axis_(axis), config_(config) {}

void HomingController::start() {
  axis_.setHomed(false);
  travelSteps_ = 0;
  backoffSteps_ = 0;
  lastStepUs_ = micros();
  setNoStepReason(NoStepReason::None);
  state_ = State::SeekFast;
}

void HomingController::update() {
  switch (state_) {
    case State::Idle:
      setNoStepReason(NoStepReason::Idle);
      return;
    case State::Done:
      setNoStepReason(NoStepReason::Done);
      return;
    case State::Error:
      setNoStepReason(NoStepReason::Error);
      return;

    case State::SeekFast:
      if (axis_.isLimitPressed()) {
        state_ = State::Backoff;
        backoffSteps_ = 0;
        setNoStepReason(NoStepReason::LimitPressedSeekFastToBackoff);
        return;
      }
      if (travelSteps_ >= mmToSteps(config_.maxTravelMm)) {
        setNoStepReason(NoStepReason::SeekFastMaxTravel);
        markError();
        return;
      }
      if (!stepDue(config_.fastSpeedMmS)) {
        return;
      }
      if (!axis_.moveOneStepForHoming(config_.direction)) {
        setNoStepReason(NoStepReason::MoveRejected);
        return;
      }
      setNoStepReason(NoStepReason::None);
      ++travelSteps_;
      return;

    case State::Backoff:
      if (!axis_.isLimitPressed()) {
        travelSteps_ = 0;
        state_ = State::SeekSlow;
        setNoStepReason(NoStepReason::BackoffLimitReleased);
        return;
      }
      if (backoffSteps_ >= mmToSteps(config_.backoffMm)) {
        setNoStepReason(NoStepReason::BackoffLimitStillOn);
        markError();
        return;
      }
      if (!stepDue(config_.fastSpeedMmS)) {
        return;
      }
      if (!axis_.moveOneStepForHoming(-config_.direction)) {
        setNoStepReason(NoStepReason::MoveRejected);
        return;
      }
      setNoStepReason(NoStepReason::None);
      ++backoffSteps_;
      return;

    case State::SeekSlow:
      if (axis_.isLimitPressed()) {
        state_ = State::SetZero;
        setNoStepReason(NoStepReason::LimitPressedSeekFastToBackoff);
        return;
      }
      if (travelSteps_ >= mmToSteps(config_.maxTravelMm)) {
        setNoStepReason(NoStepReason::SeekSlowMaxTravel);
        markError();
        return;
      }
      if (!stepDue(config_.slowSpeedMmS)) {
        return;
      }
      if (!axis_.moveOneStepForHoming(config_.direction)) {
        setNoStepReason(NoStepReason::MoveRejected);
        return;
      }
      setNoStepReason(NoStepReason::None);
      ++travelSteps_;
      return;

    case State::SetZero:
      axis_.setCurrentPositionMm(0.0F);
      axis_.setHomed(true);
      state_ = State::Done;
      setNoStepReason(NoStepReason::Done);
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
  setNoStepReason(NoStepReason::Idle);
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
    setNoStepReason(NoStepReason::Error);
    markError();
    return false;
  }

  const uint32_t intervalUs = static_cast<uint32_t>(1000000.0F / stepsPerSecond);
  const uint32_t nowUs = micros();
  if (nowUs - lastStepUs_ < intervalUs) {
    setNoStepReason(NoStepReason::StepDueWait);
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

void HomingController::setNoStepReason(NoStepReason reason) {
  lastNoStepReason_ = reason;
}

const char* HomingController::lastNoStepReasonName() const {
  switch (lastNoStepReason_) {
    case NoStepReason::Idle:
      return "IDLE";
    case NoStepReason::Done:
      return "DONE";
    case NoStepReason::Error:
      return "ERROR";
    case NoStepReason::LimitPressedSeekFastToBackoff:
      return "LIMIT_PRESSED_SEEK_FAST_TO_BACKOFF";
    case NoStepReason::BackoffLimitReleased:
      return "BACKOFF_LIMIT_RELEASED";
    case NoStepReason::StepDueWait:
      return "STEP_DUE_WAIT";
    case NoStepReason::MoveRejected:
      return "MOVE_REJECTED";
    case NoStepReason::BackoffLimitStillOn:
      return "BACKOFF_LIMIT_STILL_ON";
    case NoStepReason::SeekFastMaxTravel:
      return "SEEK_FAST_MAX_TRAVEL";
    case NoStepReason::SeekSlowMaxTravel:
      return "SEEK_SLOW_MAX_TRAVEL";
    case NoStepReason::None:
      return "NONE";
  }
  return "UNKNOWN";
}
