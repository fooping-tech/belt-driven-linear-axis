#pragma once

#include <Arduino.h>

constexpr int PIN_STEP = 5;
constexpr int PIN_DIR = 6;
constexpr int PIN_EN = -1;
constexpr int PIN_LIMIT_X_MIN = 8;
constexpr bool DIR_INVERTED = true;

constexpr uint32_t SERIAL_BAUDRATE = 115200;
constexpr uint32_t STEP_PULSE_US = 5;
constexpr uint32_t LIMIT_DEBOUNCE_MS = 30;

constexpr float STEPS_PER_MM = 80.0F;
constexpr float X_MIN_MM = 0.0F;
constexpr float RAIL_LENGTH_MM = 100.0F;
constexpr float CARRIAGE_LENGTH_MM = 40.0F;
constexpr float END_MARGIN_MM = 5.0F;
constexpr float X_MAX_TRAVEL_MM = RAIL_LENGTH_MM - CARRIAGE_LENGTH_MM - END_MARGIN_MM;
constexpr float X_MAX_MM = X_MIN_MM + X_MAX_TRAVEL_MM;

constexpr int HOMING_DIRECTION = -1;
constexpr float HOMING_FAST_MM_S = 10.0F;
constexpr float HOMING_SLOW_MM_S = 1.0F;
constexpr float HOMING_BACKOFF_MM = 2.0F;
constexpr float HOMING_MAX_TRAVEL_MM = 80.0F;

constexpr float DEFAULT_MOVE_SPEED_MM_S = 10.0F;
constexpr uint32_t BUTTON_LONG_PRESS_MS = 700;
