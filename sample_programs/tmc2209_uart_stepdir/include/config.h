#pragma once

#include <Arduino.h>

#define DRIVER_A4988 0
#define DRIVER_TMC2209 1

// Change this to DRIVER_A4988 to run the same STEP/DIR motion test without
// TMC2209 UART initialization.
#ifndef ACTIVE_DRIVER
#define ACTIVE_DRIVER DRIVER_TMC2209
#endif

constexpr uint8_t STEP_PIN = 5;
constexpr uint8_t DIR_PIN = 6;

constexpr uint8_t UART_RX_PIN = 2;
constexpr uint8_t UART_TX_PIN = 1;
constexpr uint32_t TMC_UART_BAUDRATE = 115200;
constexpr uint8_t DRIVER_ADDRESS = 0b00;
constexpr float R_SENSE = 0.11F;

// TMC2209 UART configuration.
// These names mirror the TMCStepper v0.7.3 TMC2209/TMC2208 setter APIs.
// Ranges are the register bit widths exposed by the library.

// Helper-level current and microstep settings.
// Lower this current if the motor or driver becomes hot.
constexpr uint16_t RMS_CURRENT_MA = 500;
constexpr float HOLD_MULTIPLIER = 0.50F;
constexpr uint16_t MICROSTEPS = 16;
constexpr bool TMC_USE_IHOLD_IRUN_DIRECT = false;
constexpr uint8_t TMC_IHOLD = 8;       // 0..31, used only when TMC_USE_IHOLD_IRUN_DIRECT=true
constexpr uint8_t TMC_IRUN = 16;       // 0..31, used only when TMC_USE_IHOLD_IRUN_DIRECT=true
constexpr uint8_t TMC_IHOLDDELAY = 1;  // 0..15

// Common registers.
constexpr uint8_t TMC_GSTAT_CLEAR = 0b111;  // reset, drv_err, uv_cp clear bits
constexpr uint8_t TMC_TPOWERDOWN = 20;      // 0..255
constexpr uint32_t TMC_TPWMTHRS = 0;        // 0..0xFFFFFFFF

// GCONF.
constexpr bool TMC_I_SCALE_ANALOG = false;
constexpr bool TMC_INTERNAL_RSENSE = false;
constexpr bool TMC_EN_SPREADCYCLE = false;  // false = stealthChop, true = spreadCycle
constexpr bool TMC_SHAFT = false;
constexpr bool TMC_INDEX_OTPW = false;
constexpr bool TMC_INDEX_STEP = false;
constexpr bool TMC_PDN_DISABLE = true;
constexpr bool TMC_MSTEP_REG_SELECT = true;
constexpr bool TMC_MULTISTEP_FILT = true;

// SLAVECONF.
constexpr uint8_t TMC_SENDDELAY = 2;  // 0..15

// FACTORY_CONF. Normally leave disabled; this changes trim settings.
constexpr bool TMC_WRITE_FACTORY_CONF = false;
constexpr uint8_t TMC_FCLKTRIM = 0;  // 0..31
constexpr uint8_t TMC_OTTRIM = 0;    // 0..3

// OTP_PROG. Normally leave disabled; OTP programming is one-time/nonvolatile.
constexpr bool TMC_ENABLE_OTP_PROG = false;
constexpr uint16_t TMC_OTP_PROG = 0;

// VACTUAL. Leave at 0 when using external STEP/DIR motion.
constexpr uint32_t TMC_VACTUAL = 0;

// CHOPCONF.
constexpr uint8_t TMC_TOFF = 5;    // 0..15
constexpr uint8_t TMC_HSTRT = 5;   // 0..7
constexpr uint8_t TMC_HEND = 0;    // 0..15
constexpr uint8_t TMC_TBL = 2;     // 0..3
constexpr bool TMC_VSENSE = false;
constexpr bool TMC_USE_MRES_DIRECT = false;
constexpr uint8_t TMC_MRES = 4;  // 0..15, used only when TMC_USE_MRES_DIRECT=true
constexpr bool TMC_INTPOL = true;
constexpr bool TMC_DEDGE = false;
constexpr bool TMC_DISS2G = false;
constexpr bool TMC_DISS2VS = false;

// PWMCONF.
constexpr uint8_t TMC_PWM_OFS = 36;     // 0..255
constexpr uint8_t TMC_PWM_GRAD = 14;    // 0..255
constexpr uint8_t TMC_PWM_FREQ = 1;     // 0..3
constexpr bool TMC_PWM_AUTOSCALE = true;
constexpr bool TMC_PWM_AUTOGRAD = false;
constexpr uint8_t TMC_FREEWHEEL = 0;  // 0..3
constexpr uint8_t TMC_PWM_REG = 8;    // 0..15
constexpr uint8_t TMC_PWM_LIM = 12;   // 0..15

// TMC2209 StallGuard/CoolStep.
constexpr uint32_t TMC_TCOOLTHRS = 0;  // 0..0xFFFFFFFF
constexpr uint8_t TMC_SGTHRS = 0;      // 0..255
constexpr uint8_t TMC_SEMIN = 0;       // 0..15, 0 disables CoolStep
constexpr uint8_t TMC_SEUP = 0;        // 0..3
constexpr uint8_t TMC_SEMAX = 0;       // 0..15
constexpr uint8_t TMC_SEDN = 0;        // 0..3
constexpr bool TMC_SEIMIN = false;

// GT2 belt, 20T pulley: 40 mm/rev.
// 200 full steps/rev * 16 microsteps / 40 mm = 80 steps/mm.
constexpr float STEPS_PER_MM = 80.0F;

constexpr uint8_t LIMIT_SWITCH_PIN = 8;

constexpr uint8_t DIR_FORWARD_LEVEL = HIGH;
constexpr uint8_t DIR_REVERSE_LEVEL = LOW;
constexpr bool INVERT_DIR_PIN = true;
constexpr bool INVERT_STEP_PIN = false;
constexpr bool INVERT_ENABLE_PIN = false;

// The origin is in the reverse direction. Homing moves reverse until the limit
// switch turns ON. The switch position is recorded as machine origin 0 mm, then
// the test move goes forward to TEST_TRAVEL_MM.
constexpr uint8_t HOMING_DIR_LEVEL = DIR_REVERSE_LEVEL;
constexpr uint8_t TEST_MOVE_DIR_LEVEL = DIR_FORWARD_LEVEL;
constexpr float HOMING_SPEED_MM_PER_SEC = 2.0F;
constexpr float TEST_TRAVEL_MM = 10.0F;

constexpr uint32_t STEP_PULSE_US = 5;
constexpr float ACCELERATION_MM_PER_SEC2 = 80.0F;
