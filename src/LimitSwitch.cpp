#include "LimitSwitch.h"

LimitSwitch::LimitSwitch(int pin, bool activeLow, bool usePullup, uint32_t debounceMs)
    : pin_(pin), activeLow_(activeLow), usePullup_(usePullup), debounceMs_(debounceMs) {}

void LimitSwitch::begin() {
  pinMode(pin_, usePullup_ ? INPUT_PULLUP : INPUT);
  lastRawPressed_ = isPressedRaw();
  debouncedPressed_ = lastRawPressed_;
  lastRawChangeMs_ = millis();
}

bool LimitSwitch::isPressedRaw() const {
  const int level = digitalRead(pin_);
  return activeLow_ ? level == LOW : level == HIGH;
}

bool LimitSwitch::isPressedDebounced() {
  const bool rawPressed = isPressedRaw();
  const uint32_t nowMs = millis();

  if (rawPressed != lastRawPressed_) {
    lastRawPressed_ = rawPressed;
    lastRawChangeMs_ = nowMs;
  }

  if (nowMs - lastRawChangeMs_ >= debounceMs_) {
    debouncedPressed_ = rawPressed;
  }

  return debouncedPressed_;
}

uint32_t LimitSwitch::debounceMs() const {
  return debounceMs_;
}

