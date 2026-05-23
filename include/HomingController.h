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

 private:
  bool stepDue(float speedMmS);
  void markError();
  long mmToSteps(float mm) const;

  Axis& axis_;
  HomingConfig config_;
  State state_ = State::Idle;
  uint32_t lastStepUs_ = 0;
  long travelSteps_ = 0;
  long backoffSteps_ = 0;
};

