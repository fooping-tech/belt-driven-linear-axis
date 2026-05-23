#include "StepDirDriver.h"

StepDirDriver::StepDirDriver(int stepPin, int dirPin, int enablePin, uint32_t pulseWidthUs, bool invertDirection)
    : stepPin_(stepPin),
      dirPin_(dirPin),
      enablePin_(enablePin),
      pulseWidthUs_(pulseWidthUs),
      invertDirection_(invertDirection) {}

void StepDirDriver::begin() {
  pinMode(stepPin_, OUTPUT);
  pinMode(dirPin_, OUTPUT);
  digitalWrite(stepPin_, LOW);
  digitalWrite(dirPin_, LOW);

  if (enablePin_ >= 0) {
    pinMode(enablePin_, OUTPUT);
    enable(false);
  } else {
    enabled_ = true;
  }
}

void StepDirDriver::enable(bool enabled) {
  enabled_ = enabled;
  if (enablePin_ >= 0) {
    digitalWrite(enablePin_, enabled ? LOW : HIGH);
  }
}

void StepDirDriver::setDirection(bool positive) {
  const bool pinHigh = invertDirection_ ? !positive : positive;
  digitalWrite(dirPin_, pinHigh ? HIGH : LOW);
}

void StepDirDriver::stepPulse() {
  digitalWrite(stepPin_, HIGH);
  delayMicroseconds(pulseWidthUs_);
  digitalWrite(stepPin_, LOW);
}

bool StepDirDriver::isEnabled() const {
  return enabled_;
}
