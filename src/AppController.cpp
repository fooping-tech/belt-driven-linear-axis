#include "AppController.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>

#if ACTIVE_DRIVER == DRIVER_TMC2209
#include <TMCStepper.h>
#endif

namespace {
const HomingConfig kHomingConfig = {
    HOMING_DIRECTION,
    HOMING_FAST_MM_S,
    HOMING_SLOW_MM_S,
    HOMING_BACKOFF_MM,
    HOMING_MAX_TRAVEL_MM,
    LIMIT_DEBOUNCE_MS,
};

uint16_t runtimeMicrosteps = MICROSTEPS;
float runtimeStepsPerMm = STEPS_PER_MM;

#if ACTIVE_DRIVER == DRIVER_TMC2209
HardwareSerial TmcSerial(1);
TMC2209Stepper TmcDriver(&TmcSerial, TMC_R_SENSE, TMC_DRIVER_ADDRESS);
bool tmcUartOk = false;
uint8_t tmcUartAddress = TMC_DRIVER_ADDRESS;
uint16_t tmcRuntimeRmsCurrentMa = TMC_RMS_CURRENT_MA;
bool tmcSpreadCycle = false;
uint8_t tmcConfiguredIrun = 0;
uint8_t tmcConfiguredIhold = 0;
float tmcConfiguredRunRmsMa = 0.0F;
float tmcConfiguredHoldRmsMa = 0.0F;

uint8_t refreshTmcUartStatus() {
  const uint8_t connectionResult = TmcDriver.test_connection();
  tmcUartOk = connectionResult == 0;
  return connectionResult;
}

float currentScaleToRmsMa(const uint8_t currentScale, const bool vsense) {
  const float vfs = vsense ? 0.180F : 0.325F;
  return (static_cast<float>(currentScale) + 1.0F) / 32.0F * vfs / (TMC_R_SENSE + 0.02F) / 1.41421F * 1000.0F;
}

uint8_t rmsMaToCurrentScale(const float rmsMa, const bool vsense) {
  const float vfs = vsense ? 0.180F : 0.325F;
  long currentScale = lroundf(32.0F * 1.41421F * rmsMa / 1000.0F * (TMC_R_SENSE + 0.02F) / vfs - 1.0F);
  if (currentScale < 0) {
    currentScale = 0;
  }
  if (currentScale > 31) {
    currentScale = 31;
  }
  return static_cast<uint8_t>(currentScale);
}

void applyTmc2209Current(TMC2209Stepper& driver) {
  const float targetHoldRmsMa = static_cast<float>(tmcRuntimeRmsCurrentMa) * TMC_HOLD_MULTIPLIER;
  tmcConfiguredIrun = rmsMaToCurrentScale(static_cast<float>(tmcRuntimeRmsCurrentMa), TMC_CURRENT_VSENSE);
  tmcConfiguredIhold = rmsMaToCurrentScale(targetHoldRmsMa, TMC_CURRENT_VSENSE);
  tmcConfiguredRunRmsMa = currentScaleToRmsMa(tmcConfiguredIrun, TMC_CURRENT_VSENSE);
  tmcConfiguredHoldRmsMa = currentScaleToRmsMa(tmcConfiguredIhold, TMC_CURRENT_VSENSE);

  driver.vsense(TMC_CURRENT_VSENSE);
  driver.irun(tmcConfiguredIrun);
  driver.ihold(tmcConfiguredIhold);
  driver.iholddelay(TMC_IHOLDDELAY);
}

void applyTmc2209Config(TMC2209Stepper& driver) {
  driver.begin();
  driver.GSTAT(0b111);

  driver.pdn_disable(true);
  driver.mstep_reg_select(true);
  driver.multistep_filt(true);
  driver.TPOWERDOWN(TMC_TPOWERDOWN);

  driver.toff(5);
  driver.hstrt(5);
  driver.hend(0);
  driver.tbl(2);
  driver.microsteps(runtimeMicrosteps);
  driver.intpol(true);

  driver.en_spreadCycle(tmcSpreadCycle);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(false);

  // Apply current last because CHOPCONF writes can otherwise overwrite vsense.
  applyTmc2209Current(driver);
}

void printTmc2209CurrentStatus(const char* label) {
  Serial.printf("TMC2209 current %s: target_run=%.0f mA target_hold=%.0f mA irun=%u ihold=%u iholddelay=%u vsense=%u estimated_run=%.0f mA estimated_hold=%.0f mA reported_rms=%u mA cs_actual=%u tpowerdown=%u ifcnt=%u\n",
                label,
                static_cast<float>(tmcRuntimeRmsCurrentMa),
                static_cast<float>(tmcRuntimeRmsCurrentMa) * TMC_HOLD_MULTIPLIER,
                TmcDriver.irun(),
                TmcDriver.ihold(),
                TmcDriver.iholddelay(),
                TmcDriver.vsense() ? 1 : 0,
                tmcConfiguredRunRmsMa,
                tmcConfiguredHoldRmsMa,
                TmcDriver.rms_current(),
                TmcDriver.cs_actual(),
                TMC_TPOWERDOWN,
                TmcDriver.IFCNT());
}
#endif
}

AppController::AppController()
    : driver_(PIN_STEP, PIN_DIR, PIN_EN, STEP_PULSE_US, DIR_INVERTED),
      limit_(PIN_LIMIT_X_MIN, true, true, LIMIT_DEBOUNCE_MS),
      axis_(driver_, &limit_, STEPS_PER_MM, HOMING_DIRECTION),
      homing_(axis_, kHomingConfig),
      motion_(axis_, DEFAULT_MOVE_SPEED_MM_S) {
  motion_.setAccelerationMmS2(DEFAULT_ACCELERATION_MM_S2);
}

void AppController::begin() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(SERIAL_BAUDRATE);
  delay(100);

  initDriverUart();
  driver_.begin();
  limit_.begin();
  axis_.begin();
  axis_.setSoftLimits(X_MIN_MM, X_MAX_MM);

#if ACTIVE_DRIVER == DRIVER_TMC2209
  delay(TMC_STARTUP_REAPPLY_DELAY_MS);
  Serial.printf("Reapplying TMC2209 config after StepDirDriver enable, delay=%lu ms\n",
                TMC_STARTUP_REAPPLY_DELAY_MS);
  applyTmc2209Config(TmcDriver);
  refreshTmcUartStatus();
  printTmc2209CurrentStatus("after step driver enable");
#endif

  M5.Display.setRotation(0);
  state_ = State::NotHomed;
  Serial.println("Belt axis controller ready");
  Serial.println("State: NotHomed. Long press or send 'h' to home.");
  printStatus();
  drawStatus();
}

void AppController::update() {
  M5.update();
  limit_.isPressedDebounced();
  handleButton();
  handleSerial();
  updateState();

  const uint32_t nowMs = millis();
  if (nowMs - lastDisplayMs_ >= 250) {
    lastDisplayMs_ = nowMs;
    drawStatus();
  }
}

void AppController::handleButton() {
  const bool pressed = M5.BtnA.isPressed();
  const uint32_t nowMs = millis();

  if (pressed && !wasPressed_) {
    pressStartedMs_ = nowMs;
  }

  if (!pressed && wasPressed_) {
    const uint32_t durationMs = nowMs - pressStartedMs_;
    if (durationMs >= BUTTON_LONG_PRESS_MS) {
      startHoming();
    } else if (state_ == State::Ready) {
      startMoveRelative(10.0F);
    } else if (state_ == State::Moving) {
      motion_.stop();
      state_ = State::Ready;
      Serial.println("Move stopped by button");
    } else {
      Serial.println("Short press ignored until homing completes");
    }
  }

  wasPressed_ = pressed;
}

void AppController::handleSerial() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());

    if (command == '\r' || command == '\n') {
      processSerialLine(serialLine_);
      serialLine_ = "";
      continue;
    }

    if (serialLine_.length() == 0) {
      switch (command) {
        case 'h':
        case 'H':
          startHoming();
          return;
        case '1':
          startMoveRelative(10.0F);
          return;
        case '5':
          startMoveRelative(50.0F);
          return;
        case 'b':
        case 'B':
          startMoveRelative(-10.0F);
          return;
        case 's':
        case 'S':
          if (Serial.available() == 0) {
            printStatus();
            return;
          }
          break;
      }
    }

    serialLine_ += command;
  }
}

void AppController::initDriverUart() {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  Serial.println("Initializing TMC2209 UART for STEP/DIR mode");
  TmcSerial.begin(TMC_UART_BAUDRATE, SERIAL_8N1, TMC_UART_RX_PIN, TMC_UART_TX_PIN);
  delay(20);

  Serial.printf("TMC2209 UART: RX=GPIO%u TX=GPIO%u baud=%lu addr=0b%02u R_SENSE=%.2f\n",
                TMC_UART_RX_PIN,
                TMC_UART_TX_PIN,
                TMC_UART_BAUDRATE,
                TMC_DRIVER_ADDRESS,
                TMC_R_SENSE);

  TmcDriver.begin();
  Serial.println("Applying TMC2209 startup config: pass 1");
  applyTmc2209Config(TmcDriver);
  printTmc2209CurrentStatus("after pass 1");

  delay(TMC_STARTUP_REAPPLY_DELAY_MS);
  Serial.printf("Reapplying TMC2209 startup config after %lu ms: pass 2\n",
                TMC_STARTUP_REAPPLY_DELAY_MS);
  applyTmc2209Config(TmcDriver);
  printTmc2209CurrentStatus("after pass 2");

  const uint8_t connectionResult = TmcDriver.test_connection();
  tmcUartOk = connectionResult == 0;

  Serial.printf("TMC2209 UART connection: %s (test_connection=%u)\n",
                tmcUartOk ? "OK" : "FAIL",
                connectionResult);
  Serial.printf("TMC2209 microsteps set to 1/%u, steps/mm=%.2f, rms_current=%u mA, chop=%s\n",
                runtimeMicrosteps,
                runtimeStepsPerMm,
                tmcRuntimeRmsCurrentMa,
                tmcSpreadCycle ? "spreadCycle" : "stealthChop");
  printTmc2209CurrentStatus("startup final");
#else
  Serial.println("A4988 mode: TMC2209 UART initialization skipped");
#endif
}

void AppController::processSerialLine(const String& line) {
  String trimmed = line;
  trimmed.trim();
  if (trimmed.length() == 0) {
    return;
  }

  String command = trimmed;
  command.toLowerCase();

  if (command == "s") {
    printStatus();
    return;
  }

  if (command.startsWith("profile")) {
    String args = trimmed.substring(7);
    args.trim();
    args.toLowerCase();
    if (state_ == State::Moving || state_ == State::Homing) {
      Serial.printf("Profile rejected: state=%s\n", stateName());
      return;
    }
    if (args == "trap" || args == "trapezoid") {
      motion_.setProfile(MotionController::Profile::Trapezoid);
      Serial.println("Motion profile set: trap");
      return;
    }
    if (args == "direct") {
      motion_.setProfile(MotionController::Profile::Direct);
      Serial.println("Motion profile set: direct");
      return;
    }
    Serial.println("Usage: profile trap|direct");
    return;
  }

  if (command.startsWith("accel") || command.startsWith("a")) {
    float accelerationMmS2 = 0.0F;
    if (!parseAccelCommand(trimmed, accelerationMmS2) || !validateAcceleration(accelerationMmS2)) {
      Serial.printf("Usage: a <accel_mm_s2>, accel range %.2f..%.2f mm/s^2\n",
                    TEST_ACCEL_MIN_MM_S2,
                    TEST_ACCEL_MAX_MM_S2);
      return;
    }
    if (state_ == State::Moving || state_ == State::Homing) {
      Serial.printf("Acceleration rejected: state=%s\n", stateName());
      return;
    }

    motion_.setAccelerationMmS2(accelerationMmS2);
    Serial.printf("Acceleration set: %.2f mm/s^2\n", motion_.accelerationMmS2());
    return;
  }

  if (command.startsWith("chop") || command.startsWith("mode")) {
    String args = command.startsWith("chop") ? trimmed.substring(4) : trimmed.substring(4);
    args.trim();
    args.toLowerCase();
    if (args == "spread" || args == "spreadcycle") {
      setChopMode(true);
      return;
    }
    if (args == "stealth" || args == "stealthchop") {
      setChopMode(false);
      return;
    }
    Serial.println("Usage: chop stealth|spread, mode stealth|spread");
    return;
  }

  if (command.startsWith("current") || command.startsWith("i")) {
    uint16_t currentMa = 0;
    if (!parseUnsignedCommand(trimmed, currentMa)) {
      Serial.printf("Usage: i <current_mA>, current range %u..%u mA\n",
                    TEST_CURRENT_MIN_MA,
                    TEST_CURRENT_MAX_MA);
      return;
    }
    setRuntimeCurrent(currentMa);
    return;
  }

  if (command.startsWith("microstep")) {
    uint16_t microsteps = 0;
    if (!parseUnsignedCommand(trimmed, microsteps)) {
      Serial.println("Usage: microstep 16|8");
      return;
    }
    setMicrosteps(microsteps);
    return;
  }

  if (trimmed.startsWith("m") || trimmed.startsWith("M")) {
    float distanceMm = 0.0F;
    float speedMmS = 0.0F;
    if (!parseMoveCommand(trimmed, distanceMm, speedMmS)) {
      printMoveUsage();
      return;
    }
    startMoveRelative(distanceMm, speedMmS);
    return;
  }

  if (command.startsWith("v") || command.startsWith("speed")) {
    float speedMmS = 0.0F;
    if (!parseSpeedCommand(trimmed, speedMmS) || !validateMoveSpeed(speedMmS)) {
      Serial.printf("Usage: v <speed_mm_s>, speed range %.2f..%.2f mm/s\n",
                    TEST_MOVE_MIN_SPEED_MM_S,
                    TEST_MOVE_MAX_SPEED_MM_S);
      return;
    }

    defaultMoveSpeedMmS_ = speedMmS;
    Serial.printf("Default move speed set: %.2f mm/s\n", defaultMoveSpeedMmS_);
    return;
  }

  if (trimmed.equalsIgnoreCase("on") || trimmed.equalsIgnoreCase("enable")) {
    setMotorPower(true);
    return;
  }

  if (trimmed.equalsIgnoreCase("off") || trimmed.equalsIgnoreCase("disable")) {
    setMotorPower(false);
    return;
  }

  Serial.printf("Unknown command '%s'. Use h, 1, 5, b, s, on, off, v <mm/s>, a <mm/s2>, profile trap|direct, i <mA>, mode stealth|spread, microstep 16|8, or m <mm> [mm/s].\n", trimmed.c_str());
}

bool AppController::parseSpeedCommand(const String& line, float& speedMmS) const {
  String command = line;
  command.toLowerCase();

  String args;
  if (command.startsWith("speed")) {
    args = line.substring(5);
  } else {
    args = line.substring(1);
  }

  args.trim();
  if (args.length() == 0 || args.length() >= 32) {
    return false;
  }

  char buffer[32];
  args.toCharArray(buffer, sizeof(buffer));

  char* cursor = buffer;
  while (isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }

  char* end = nullptr;
  speedMmS = strtof(cursor, &end);
  if (end == cursor) {
    return false;
  }

  cursor = end;
  while (isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }

  return *cursor == '\0';
}

bool AppController::parseAccelCommand(const String& line, float& accelerationMmS2) const {
  String command = line;
  command.toLowerCase();

  String args;
  if (command.startsWith("accel")) {
    args = line.substring(5);
  } else {
    args = line.substring(1);
  }

  args.trim();
  if (args.length() == 0 || args.length() >= 32) {
    return false;
  }

  char buffer[32];
  args.toCharArray(buffer, sizeof(buffer));

  char* cursor = buffer;
  while (isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }

  char* end = nullptr;
  accelerationMmS2 = strtof(cursor, &end);
  if (end == cursor) {
    return false;
  }

  cursor = end;
  while (isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }

  return *cursor == '\0';
}

bool AppController::parseUnsignedCommand(const String& line, uint16_t& value) const {
  int start = 1;
  String command = line;
  command.toLowerCase();
  if (command.startsWith("current")) {
    start = 7;
  } else if (command.startsWith("microstep")) {
    start = 9;
  }

  String args = line.substring(start);
  args.trim();
  if (args.length() == 0 || args.length() >= 16) {
    return false;
  }

  char buffer[16];
  args.toCharArray(buffer, sizeof(buffer));

  char* cursor = buffer;
  while (isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }

  char* end = nullptr;
  const unsigned long parsed = strtoul(cursor, &end, 10);
  if (end == cursor || parsed > 65535UL) {
    return false;
  }

  cursor = end;
  while (isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }
  if (*cursor != '\0') {
    return false;
  }

  value = static_cast<uint16_t>(parsed);
  return true;
}

bool AppController::parseMoveCommand(const String& line, float& distanceMm, float& speedMmS) const {
  String args = line.substring(1);
  args.trim();
  if (args.length() == 0 || args.length() >= 64) {
    return false;
  }

  char buffer[64];
  args.toCharArray(buffer, sizeof(buffer));

  char* cursor = buffer;
  while (isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }

  char* end = nullptr;
  distanceMm = strtof(cursor, &end);
  if (end == cursor) {
    return false;
  }

  cursor = end;
  while (isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }

  if (*cursor == '\0') {
    speedMmS = defaultMoveSpeedMmS_;
    return true;
  }

  speedMmS = strtof(cursor, &end);
  if (end == cursor) {
    return false;
  }

  cursor = end;
  while (isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }

  return *cursor == '\0';
}

bool AppController::validateMoveSpeed(float speedMmS) const {
  if (!isfinite(speedMmS)) {
    Serial.println("Speed rejected: speed must be a finite number");
    return false;
  }

  if (speedMmS < TEST_MOVE_MIN_SPEED_MM_S || speedMmS > TEST_MOVE_MAX_SPEED_MM_S) {
    Serial.printf("Speed rejected: %.2f mm/s is outside %.2f..%.2f mm/s\n",
                  speedMmS,
                  TEST_MOVE_MIN_SPEED_MM_S,
                  TEST_MOVE_MAX_SPEED_MM_S);
    return false;
  }

  return true;
}

bool AppController::validateAcceleration(float accelerationMmS2) const {
  if (!isfinite(accelerationMmS2)) {
    Serial.println("Acceleration rejected: acceleration must be a finite number");
    return false;
  }

  if (accelerationMmS2 < TEST_ACCEL_MIN_MM_S2 || accelerationMmS2 > TEST_ACCEL_MAX_MM_S2) {
    Serial.printf("Acceleration rejected: %.2f mm/s^2 is outside %.2f..%.2f mm/s^2\n",
                  accelerationMmS2,
                  TEST_ACCEL_MIN_MM_S2,
                  TEST_ACCEL_MAX_MM_S2);
    return false;
  }

  return true;
}

bool AppController::validateMoveRequest(float distanceMm, float speedMmS) const {
  if (!motorPowerEnabled_) {
    Serial.println("Move rejected: motor power is OFF. Send 'on' first, then home again.");
    return false;
  }

#if ACTIVE_DRIVER == DRIVER_TMC2209
  refreshTmcUartStatus();
  if (!tmcUartOk) {
    Serial.println("Move rejected: TMC2209 UART is not OK, microstep setting is not verified");
    return false;
  }
#endif

  if (!isfinite(distanceMm) || !isfinite(speedMmS)) {
    Serial.println("Move rejected: distance and speed must be finite numbers");
    printMoveUsage();
    return false;
  }

  if (!validateMoveSpeed(speedMmS)) {
    printMoveUsage();
    return false;
  }

  if (state_ != State::Ready) {
    Serial.printf("Move rejected: state=%s, homed=%s\n", stateName(), axis_.isHomed() ? "true" : "false");
    return false;
  }

  const float targetMm = axis_.currentPositionMm() + distanceMm;
  if (!axis_.isWithinSoftLimit(targetMm)) {
    Serial.printf("Move rejected: target %.2f mm is outside %.2f..%.2f mm\n", targetMm, X_MIN_MM, X_MAX_MM);
    return false;
  }

  return true;
}

void AppController::printMoveUsage() const {
  Serial.printf("Usage: m <distance_mm> [speed_mm_s], v <speed_mm_s>, a <accel_mm_s2>, profile trap|direct, speed range %.2f..%.2f mm/s, default %.2f mm/s\n",
                TEST_MOVE_MIN_SPEED_MM_S,
                TEST_MOVE_MAX_SPEED_MM_S,
                defaultMoveSpeedMmS_);
}

void AppController::setMotorPower(bool enabled) {
  motion_.stop();
  if (state_ == State::Homing) {
    homing_.reset();
  }

  motorPowerEnabled_ = enabled;
  driver_.enable(enabled);

#if ACTIVE_DRIVER == DRIVER_TMC2209
  if (enabled) {
    applyTmc2209Config(TmcDriver);
    refreshTmcUartStatus();
  } else {
    TmcDriver.toff(0);
  }
#endif

  if (!enabled) {
    axis_.setHomed(false);
    state_ = State::NotHomed;
    Serial.println("Motor power OFF: driver output disabled. Position is no longer trusted; home again after ON.");
  } else {
    state_ = axis_.isHomed() ? State::Ready : State::NotHomed;
    Serial.println("Motor power ON: driver output enabled. Home before normal movement.");
  }

  printStatus();
}

void AppController::setRuntimeCurrent(uint16_t currentMa) {
  if (state_ == State::Moving || state_ == State::Homing) {
    Serial.printf("Current rejected: state=%s\n", stateName());
    return;
  }
  if (currentMa < TEST_CURRENT_MIN_MA || currentMa > TEST_CURRENT_MAX_MA) {
    Serial.printf("Current rejected: %u mA is outside %u..%u mA\n",
                  currentMa,
                  TEST_CURRENT_MIN_MA,
                  TEST_CURRENT_MAX_MA);
    return;
  }

#if ACTIVE_DRIVER == DRIVER_TMC2209
  refreshTmcUartStatus();
  if (!tmcUartOk) {
    Serial.println("Current rejected: TMC2209 UART is not OK");
    return;
  }
  tmcRuntimeRmsCurrentMa = currentMa;
  applyTmc2209Current(TmcDriver);
  refreshTmcUartStatus();
  Serial.printf("TMC2209 current set: requestedRms=%u mA estimatedRun=%.0f mA estimatedHold=%.0f mA uart=%s\n",
                tmcRuntimeRmsCurrentMa,
                tmcConfiguredRunRmsMa,
                tmcConfiguredHoldRmsMa,
                tmcUartOk ? "OK" : "FAIL");
#else
  Serial.println("Current command ignored: ACTIVE_DRIVER is not TMC2209");
#endif
}

void AppController::setChopMode(bool spreadCycle) {
  if (state_ == State::Moving || state_ == State::Homing) {
    Serial.printf("Chop mode rejected: state=%s\n", stateName());
    return;
  }

#if ACTIVE_DRIVER == DRIVER_TMC2209
  refreshTmcUartStatus();
  if (!tmcUartOk) {
    Serial.println("Chop mode rejected: TMC2209 UART is not OK");
    return;
  }
  tmcSpreadCycle = spreadCycle;
  TmcDriver.en_spreadCycle(tmcSpreadCycle);
  refreshTmcUartStatus();
  Serial.printf("TMC2209 chop mode set: %s uart=%s\n",
                tmcSpreadCycle ? "spreadCycle" : "stealthChop",
                tmcUartOk ? "OK" : "FAIL");
#else
  Serial.println("Chop mode command ignored: ACTIVE_DRIVER is not TMC2209");
#endif
}

void AppController::setMicrosteps(uint16_t microsteps) {
  if (state_ == State::Moving || state_ == State::Homing) {
    Serial.printf("Microstep rejected: state=%s\n", stateName());
    return;
  }
  if (microsteps != 8 && microsteps != 16) {
    Serial.println("Microstep rejected: use 16 or 8 for this diagnostic");
    return;
  }

#if ACTIVE_DRIVER == DRIVER_TMC2209
  refreshTmcUartStatus();
  if (!tmcUartOk) {
    Serial.println("Microstep rejected: TMC2209 UART is not OK");
    return;
  }
  runtimeMicrosteps = microsteps;
  runtimeStepsPerMm = (MOTOR_FULL_STEPS_PER_REV * runtimeMicrosteps) / PULLEY_TRAVEL_MM_PER_REV;
  axis_.setStepsPerMm(runtimeStepsPerMm);
  axis_.setHomed(false);
  motion_.stop();
  TmcDriver.microsteps(runtimeMicrosteps);
  refreshTmcUartStatus();
  state_ = State::NotHomed;
  Serial.printf("TMC2209 microstep set: 1/%u stepsPerMm=%.2f uart=%s. Position is no longer trusted; home again.\n",
                runtimeMicrosteps,
                runtimeStepsPerMm,
                tmcUartOk ? "OK" : "FAIL");
#else
  Serial.println("Microstep command ignored: ACTIVE_DRIVER is not TMC2209");
#endif
}

void AppController::startHoming() {
  if (!motorPowerEnabled_) {
    Serial.println("Homing rejected: motor power is OFF. Send 'on' first.");
    return;
  }

#if ACTIVE_DRIVER == DRIVER_TMC2209
  refreshTmcUartStatus();
  if (!tmcUartOk) {
    state_ = State::Error;
    Serial.println("Homing rejected: TMC2209 UART is not OK, microstep setting is not verified");
    return;
  }
#endif

  if (state_ == State::Homing || state_ == State::Moving) {
    Serial.printf("Cannot start homing while state=%s\n", stateName());
    return;
  }

  homing_.reset();
  homing_.start();
  lastHomingLogState_ = homing_.state();
  state_ = State::Homing;
  Serial.printf("Homing started: state=%s direction=%d fast=%.2fmm/s slow=%.2fmm/s backoff=%.2fmm max=%.2fmm\n",
                homing_.stateName(),
                HOMING_DIRECTION,
                HOMING_FAST_MM_S,
                HOMING_SLOW_MM_S,
                HOMING_BACKOFF_MM,
                HOMING_MAX_TRAVEL_MM);
}

void AppController::startMoveRelative(float mm) {
  startMoveRelative(mm, defaultMoveSpeedMmS_);
}

void AppController::startMoveRelative(float mm, float speedMmS) {
  if (!validateMoveRequest(mm, speedMmS)) {
    return;
  }

  const float targetMm = axis_.currentPositionMm() + mm;
  if (!motion_.moveRelativeMm(mm, speedMmS)) {
    state_ = State::Error;
    Serial.println("Move rejected by motion controller");
    return;
  }

  state_ = State::Moving;
  Serial.printf("Move started: delta=%.2f mm speed=%.2f mm/s accel=%.2f mm/s^2 profile=%s target=%.2f mm softLimit=%.2f..%.2f mm microsteps=1/%u stepsPerMm=%.2f\n",
                mm,
                speedMmS,
                motion_.accelerationMmS2(),
                motion_.profileName(),
                targetMm,
                X_MIN_MM,
                X_MAX_MM,
                runtimeMicrosteps,
                axis_.stepsPerMm());
}

void AppController::updateState() {
  switch (state_) {
    case State::Boot:
      state_ = State::NotHomed;
      break;

    case State::NotHomed:
      break;

    case State::Homing:
      homing_.update();
      if (homing_.state() != lastHomingLogState_) {
        lastHomingLogState_ = homing_.state();
        Serial.printf("Homing state: %s pos=%.2fmm steps=%ld limit=%s\n",
                      homing_.stateName(),
                      axis_.currentPositionMm(),
                      axis_.currentPositionSteps(),
                      limit_.isPressedDebounced() ? "ON" : "OFF");
      }
      if (homing_.isDone()) {
        state_ = State::Ready;
        Serial.println("Homing complete. X=0.00 mm");
        printStatus();
      } else if (homing_.hasError()) {
        state_ = State::Error;
        Serial.println("Homing error");
        printStatus();
      }
      break;

    case State::Ready:
      if (axis_.isLimitPressed() && axis_.currentPositionMm() > X_MIN_MM + 0.5F) {
        state_ = State::Error;
        Serial.println("Unexpected limit switch ON while ready");
      }
      break;

    case State::Moving:
      motion_.update();
      if (!motion_.isMoving() && !motion_.hasError()) {
        state_ = State::Ready;
        Serial.println("Move complete");
        printStatus();
      } else if (motion_.hasError()) {
        state_ = State::Error;
        Serial.printf("Motion error detail: pos=%.4fmm steps=%ld target=%.4fmm targetSteps=%ld remainingSteps=%ld limitRaw=%s limitDebounced=%s\n",
                      axis_.currentPositionMm(),
                      axis_.currentPositionSteps(),
                      motion_.targetMm(),
                      motion_.targetSteps(),
                      motion_.remainingSteps(),
                      limit_.isPressedRaw() ? "ON" : "OFF",
                      limit_.isPressedDebounced() ? "ON" : "OFF");
        Serial.println("Motion error: limit switch or soft limit stopped movement");
        printStatus();
      }
      break;

    case State::Error:
      break;
  }
}

void AppController::printStatus() {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  const uint8_t connectionResult = refreshTmcUartStatus();
#endif

  Serial.printf("state=%s homing=%s motion=%s pos=%.2fmm steps=%ld speed=%.2fmm/s defaultSpeed=%.2fmm/s accel=%.2fmm/s^2 profile=%s homed=%s limitRaw=%s limitDebounced=%s\n",
                stateName(),
                homing_.stateName(),
                motion_.stateName(),
                axis_.currentPositionMm(),
                axis_.currentPositionSteps(),
                motion_.speedMmS(),
                defaultMoveSpeedMmS_,
                motion_.accelerationMmS2(),
                motion_.profileName(),
                axis_.isHomed() ? "true" : "false",
                limit_.isPressedRaw() ? "ON" : "OFF",
                limit_.isPressedDebounced() ? "ON" : "OFF");
  Serial.printf("motorPower=%s stepDriverEnabled=%s\n",
                motorPowerEnabled_ ? "ON" : "OFF",
                driver_.isEnabled() ? "true" : "false");
#if ACTIVE_DRIVER == DRIVER_TMC2209
  Serial.printf("tmc2209_uart=%s test_connection=%u microsteps=1/%u stepsPerMm=%.2f runtimeCurrent=%u mA chopMode=%s\n",
                tmcUartOk ? "OK" : "FAIL",
                connectionResult,
                runtimeMicrosteps,
                axis_.stepsPerMm(),
                tmcRuntimeRmsCurrentMa,
                tmcSpreadCycle ? "spreadCycle" : "stealthChop");
  Serial.printf("tmc2209_uart_address=0b%02u rx=GPIO%u tx=GPIO%u\n",
                tmcUartAddress,
                TMC_UART_RX_PIN,
                TMC_UART_TX_PIN);
  Serial.printf("tmc2209_current_config=requestedRms=%u mA holdMultiplier=%.2f targetHold=%.0f mA irun=%u ihold=%u iholddelay=%u vsense=%u estimatedRun=%.0f mA estimatedHold=%.0f mA reportedRms=%u mA csActual=%u tpowerdown=%u ifcnt=%u\n",
                tmcRuntimeRmsCurrentMa,
                TMC_HOLD_MULTIPLIER,
                static_cast<float>(tmcRuntimeRmsCurrentMa) * TMC_HOLD_MULTIPLIER,
                TmcDriver.irun(),
                TmcDriver.ihold(),
                TmcDriver.iholddelay(),
                TmcDriver.vsense() ? 1 : 0,
                tmcConfiguredRunRmsMa,
                tmcConfiguredHoldRmsMa,
                TmcDriver.rms_current(),
                TmcDriver.cs_actual(),
                TMC_TPOWERDOWN,
                TmcDriver.IFCNT());
#endif
}

void AppController::drawStatus() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(middle_center);

  M5.Display.setTextSize(2);
  M5.Display.drawString("Belt Axis", M5.Display.width() / 2, 12);

  M5.Display.setTextSize(1);
  M5.Display.drawString(String("State: ") + stateName(), M5.Display.width() / 2, 34);
  M5.Display.drawString(String("Home: ") + homing_.stateName(), M5.Display.width() / 2, 50);
  M5.Display.drawString(String("X: ") + String(axis_.currentPositionMm(), 2) + " mm", M5.Display.width() / 2, 66);
  M5.Display.drawString(String("Default: ") + String(defaultMoveSpeedMmS_, 1) + " mm/s", M5.Display.width() / 2, 82);
  M5.Display.drawString(String("Limit: ") + (limit_.isPressedDebounced() ? "ON" : "OFF"), M5.Display.width() / 2, 98);

  const char* footer = "Serial: v speed";
  if (state_ == State::NotHomed) {
    footer = "Long press: home";
  } else if (state_ == State::Error) {
    footer = "Error: send h";
  }
  M5.Display.drawString(footer, M5.Display.width() / 2, 112);
}

const char* AppController::stateName() const {
  switch (state_) {
    case State::Boot:
      return "Boot";
    case State::NotHomed:
      return "NotHomed";
    case State::Homing:
      return "Homing";
    case State::Ready:
      return "Ready";
    case State::Moving:
      return "Moving";
    case State::Error:
      return "Error";
  }
  return "Unknown";
}
