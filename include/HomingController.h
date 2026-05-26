#pragma once

#include <Arduino.h>

#include "Axis.h"

struct HomingConfig {
  int direction;
  float fastSpeedMmS;
  float slowSpeedMmS;
  float backoffMm;
  float maxTravelMm;
  uint32_t debounceMs;
};

class HomingController {
 public:
  enum class State : uint8_t {
    Idle,
    SeekFast,
    Backoff,
    SeekSlow,
    SetZero,
    Done,
    Error,
  };

  HomingController(Axis& axis, const HomingConfig& config);

  void start();
  void update();
  bool isBusy() const;
  bool isDone() const;
  bool hasError() const;
  void reset();
  State state() const;
  const char* stateName() const;
  const char* lastNoStepReasonName() const;

 private:
  enum class NoStepReason : uint8_t {
    Idle,
    Done,
    Error,
    LimitPressedSeekFastToBackoff,
    BackoffLimitReleased,
    StepDueWait,
    MoveRejected,
    BackoffLimitStillOn,
    SeekFastMaxTravel,
    SeekSlowMaxTravel,
    None,
  };

  bool stepDue(float speedMmS);
  void markError();
  long mmToSteps(float mm) const;
  void setNoStepReason(NoStepReason reason);

  Axis& axis_;
  HomingConfig config_;
  State state_ = State::Idle;
  uint32_t lastStepUs_ = 0;
  long travelSteps_ = 0;
  long backoffSteps_ = 0;
  NoStepReason lastNoStepReason_ = NoStepReason::Idle;
};
