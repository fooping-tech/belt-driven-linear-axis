#pragma once

#include <Arduino.h>

#define DRIVER_A4988 0
#define DRIVER_TMC2209 1

#ifndef ACTIVE_DRIVER
#define ACTIVE_DRIVER DRIVER_TMC2209
#endif

constexpr int PIN_STEP = 5;
constexpr int PIN_DIR = 6;
constexpr int PIN_EN = -1;
constexpr int PIN_LIMIT_X_MIN = 8;
constexpr bool DIR_INVERTED = true;

constexpr uint8_t TMC_UART_RX_PIN = 2;
constexpr uint8_t TMC_UART_TX_PIN = 1;
constexpr uint32_t TMC_UART_BAUDRATE = 115200;
constexpr uint8_t TMC_DRIVER_ADDRESS = 0b00;
constexpr float TMC_R_SENSE = 0.11F;
constexpr uint16_t TMC_RMS_CURRENT_MA = 500;
constexpr float TMC_HOLD_MULTIPLIER = 0.5F;
constexpr bool TMC_CURRENT_VSENSE = false;
constexpr uint8_t TMC_IHOLDDELAY = 1;
constexpr uint8_t TMC_TPOWERDOWN = 20;
constexpr uint32_t TMC_STARTUP_REAPPLY_DELAY_MS = 100;

constexpr uint32_t SERIAL_BAUDRATE = 115200;
constexpr uint32_t STEP_PULSE_US = 5;
constexpr uint32_t LIMIT_DEBOUNCE_MS = 30;

constexpr float BELT_PITCH_MM = 2.0F;
constexpr uint16_t PULLEY_TEETH = 20;
constexpr uint16_t MOTOR_FULL_STEPS_PER_REV = 200;
constexpr uint16_t MICROSTEPS = 16;
constexpr float PULLEY_TRAVEL_MM_PER_REV = BELT_PITCH_MM * PULLEY_TEETH;
constexpr float STEPS_PER_MM = (MOTOR_FULL_STEPS_PER_REV * MICROSTEPS) / PULLEY_TRAVEL_MM_PER_REV;
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
constexpr float TEST_MOVE_MIN_SPEED_MM_S = 0.1F;
constexpr float TEST_MOVE_MAX_SPEED_MM_S = 50.0F;
constexpr uint32_t BUTTON_LONG_PRESS_MS = 700;
