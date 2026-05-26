#pragma once

#include <Arduino.h>

class StepDirDriver {
 public:
  StepDirDriver(int stepPin, int dirPin, int enablePin, uint32_t pulseWidthUs, bool invertDirection = false);

  void begin();
  void enable(bool enabled);
  void setDirection(bool positive);
  void stepPulse();
  bool isEnabled() const;
  uint32_t stepPulseCount() const;
  uint32_t lastStepPulseUs() const;

 private:
  int stepPin_;
  int dirPin_;
  int enablePin_;
  uint32_t pulseWidthUs_;
  bool invertDirection_;
  bool enabled_ = false;
  uint32_t stepPulseCount_ = 0;
  uint32_t lastStepPulseUs_ = 0;
};
