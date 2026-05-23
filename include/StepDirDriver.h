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

 private:
  int stepPin_;
  int dirPin_;
  int enablePin_;
  uint32_t pulseWidthUs_;
  bool invertDirection_;
  bool enabled_ = false;
};
