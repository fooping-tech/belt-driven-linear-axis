#pragma once

#include <Arduino.h>

class LimitSwitch {
 public:
  LimitSwitch(int pin, bool activeLow = true, bool usePullup = true, uint32_t debounceMs = 30);

  void begin();
  bool isPressedRaw() const;
  bool isPressedDebounced();
  uint32_t debounceMs() const;

 private:
  int pin_;
  bool activeLow_;
  bool usePullup_;
  uint32_t debounceMs_;
  bool debouncedPressed_ = false;
  bool lastRawPressed_ = false;
  uint32_t lastRawChangeMs_ = 0;
};

