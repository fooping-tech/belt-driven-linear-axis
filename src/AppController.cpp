#include "AppController.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <esp_system.h>

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
bool sgCaptureActive = false;
uint32_t sgCaptureStartedMs = 0;
const char* homeEndLimitState = "NA";
const char* moveStartLimitState = "NA";
const char* moveEndLimitState = "NA";
const char* timeoutLimitState = "NA";
const char* limitFirstTriggerTiming = "NONE";
uint32_t limitTransitionCount = 0;
bool lastMoveLimitState = false;
bool moveLimitStateInitialized = false;

struct MotorMelodyNote {
  uint16_t frequencyHz;
  uint16_t durationMs;
};

constexpr MotorMelodyNote kStartupMotorMelody[] = {
    {523, 90},
    {659, 90},
    {784, 120},
    {1047, 180},
};

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
uint8_t sgThreshold = TMC_SGTHRS_DEFAULT;
uint32_t sgTcoolThreshold = TMC_TCOOLTHRS_DEFAULT;
bool sgCurrentValid = false;
uint16_t sgCurrent = 0;
bool sgStatsHaveValidSample = false;
uint16_t sgMinValid = 0;
uint16_t sgMaxValid = 0;
uint64_t sgSumValid = 0;
uint32_t sgCount = 0;
uint32_t sgValidCount = 0;
uint32_t sgZeroCount = 0;
uint32_t sgLowCount50 = 0;
uint32_t sgLowCount100 = 0;
uint32_t sgLowCount150 = 0;
bool sgSamplingEnabled = SG_POLLING_ENABLED_DEFAULT;
uint32_t sgSampleIntervalMs = SG_UPDATE_INTERVAL_MS;
bool sgLogEnabled = false;
uint32_t sgLogIntervalMs = SG_LOG_INTERVAL_MS_DEFAULT;
uint32_t lastSgUpdateMs = 0;
uint32_t lastSgLogMs = 0;
uint32_t lastSgErrorMs = 0;

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
  driver.semin(0);
  driver.SGTHRS(sgThreshold);
  driver.TCOOLTHRS(sgTcoolThreshold);

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
      as5600_(PIN_AS5600_SDA, PIN_AS5600_SCL),
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
  resetReason_ = static_cast<int>(esp_reset_reason());
  if (HEARTBEAT_ENABLED) {
    pinMode(PIN_HEARTBEAT, OUTPUT);
    digitalWrite(PIN_HEARTBEAT, heartbeatState_ ? HIGH : LOW);
  }
  lastLoopTickUs_ = micros();
  lastHeartbeatToggleMs_ = millis();
  printResetReason();
  initAs5600();

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
  playStartupMotorMelody();

  M5.Display.setRotation(0);
  state_ = State::NotHomed;
  Serial.println("Belt axis controller ready");
  Serial.println("State: NotHomed. Long press or send 'h' to home.");
  printStatus();
  drawStatus();
}

void AppController::update() {
  updateHeartbeatAndLoopStats();
  M5.update();
  limit_.isPressedDebounced();
  handleButton();
  handleSerial();
  updateState();
  updateStallGuardStats();

  const uint32_t nowMs = millis();
  if (nowMs - lastDisplayMs_ >= 250) {
    lastDisplayMs_ = nowMs;
    drawStatus();
  }
}

void AppController::updateHeartbeatAndLoopStats() {
  const uint32_t nowUs = micros();
  if (lastLoopTickUs_ != 0) {
    lastLoopGapUs_ = nowUs - lastLoopTickUs_;
    if (lastLoopGapUs_ > maxLoopGapUs_) {
      maxLoopGapUs_ = lastLoopGapUs_;
    }
  }
  lastLoopTickUs_ = nowUs;

  if (!HEARTBEAT_ENABLED) {
    return;
  }
  const uint32_t nowMs = millis();
  if (nowMs - lastHeartbeatToggleMs_ >= HEARTBEAT_TOGGLE_INTERVAL_MS) {
    lastHeartbeatToggleMs_ = nowMs;
    heartbeatState_ = !heartbeatState_;
    digitalWrite(PIN_HEARTBEAT, heartbeatState_ ? HIGH : LOW);
  }
}

void AppController::printLoopDiagnostics() const {
  Serial.printf("loop_diag,heartbeat_enabled=%u,heartbeat_pin=%d,last_loop_gap_us=%lu,max_loop_gap_us=%lu,reset_reason=%s\n",
                HEARTBEAT_ENABLED ? 1 : 0,
                PIN_HEARTBEAT,
                static_cast<unsigned long>(lastLoopGapUs_),
                static_cast<unsigned long>(maxLoopGapUs_),
                resetReasonName());
}

void AppController::printResetReason() const {
  Serial.printf("reset_reason=%s reset_reason_code=%d\n", resetReasonName(), resetReason_);
}

const char* AppController::resetReasonName() const {
  switch (static_cast<esp_reset_reason_t>(resetReason_)) {
    case ESP_RST_POWERON:
      return "POWERON";
    case ESP_RST_EXT:
      return "EXTERNAL";
    case ESP_RST_SW:
      return "SOFTWARE";
    case ESP_RST_PANIC:
      return "PANIC";
    case ESP_RST_INT_WDT:
      return "WATCHDOG";
    case ESP_RST_TASK_WDT:
      return "WATCHDOG";
    case ESP_RST_WDT:
      return "WATCHDOG";
    case ESP_RST_DEEPSLEEP:
      return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT";
    case ESP_RST_SDIO:
      return "SDIO";
    default:
      return "UNKNOWN";
  }
}

void AppController::playStartupMotorMelody() {
  if (!STARTUP_MOTOR_MELODY_ENABLED || !driver_.isEnabled()) {
    return;
  }

#if ACTIVE_DRIVER == DRIVER_TMC2209
  const uint16_t restoreMicrosteps = runtimeMicrosteps;
  const bool restoreSpreadCycle = tmcSpreadCycle;
  const uint16_t restoreCurrentMa = tmcRuntimeRmsCurrentMa;
  tmcRuntimeRmsCurrentMa = STARTUP_MOTOR_MELODY_CURRENT_MA;
  applyTmc2209Current(TmcDriver);
  TmcDriver.microsteps(STARTUP_MOTOR_MELODY_MICROSTEPS);
  TmcDriver.en_spreadCycle(STARTUP_MOTOR_MELODY_SPREADCYCLE);
  refreshTmcUartStatus();
#endif

  Serial.printf("startup_motor_melody=on microsteps=1/%u chop=%s current_ma=%u\n",
                STARTUP_MOTOR_MELODY_MICROSTEPS,
                STARTUP_MOTOR_MELODY_SPREADCYCLE ? "spreadCycle" : "stealthChop",
                STARTUP_MOTOR_MELODY_CURRENT_MA);
  bool directionPositive = HOMING_DIRECTION < 0;
  for (const MotorMelodyNote& note : kStartupMotorMelody) {
    if (note.frequencyHz == 0) {
      delay(note.durationMs);
      continue;
    }

    const uint32_t periodUs = 1000000UL / note.frequencyHz;
    const uint32_t startedMs = millis();
    while (millis() - startedMs < note.durationMs) {
      driver_.setDirection(directionPositive);
      delayMicroseconds(2);
      driver_.stepPulse();
      directionPositive = !directionPositive;
      delayMicroseconds(periodUs);
    }
    delay(STARTUP_MOTOR_MELODY_NOTE_GAP_MS);
  }

#if ACTIVE_DRIVER == DRIVER_TMC2209
  tmcRuntimeRmsCurrentMa = restoreCurrentMa;
  applyTmc2209Current(TmcDriver);
  TmcDriver.microsteps(restoreMicrosteps);
  TmcDriver.en_spreadCycle(restoreSpreadCycle);
  refreshTmcUartStatus();
#endif
  Serial.println("startup_motor_melody=done");
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
      sgCaptureActive = false;
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

  if (command == "diag") {
    printDiagnosticStatus();
    return;
  }

  if (command == "io") {
    printIoStatus();
    return;
  }

  if (command == "as5600") {
    printAs5600Status();
    return;
  }

  if (command == "angle") {
    printAs5600Angle();
    return;
  }

  if (command == "as5600bb") {
    printAs5600BitBangStatus();
    return;
  }

  if (command == "i2cscan") {
    printI2cScan();
    return;
  }

  if (command == "i2cscanbb") {
    printI2cBitBangScan();
    return;
  }

  if (command == "i2cpins") {
    runI2cPinPulseTest();
    return;
  }

  if (command == "motortest") {
    runMotorStepTest();
    return;
  }

  if (command == "sg") {
    printStallGuardStatus();
    return;
  }

  if (command == "mt") {
    if (state_ == State::Moving) {
      timeoutLimitState = limit_.isPressedDebounced() ? "ON" : "OFF";
    }
    printMotionTimingSummary();
    return;
  }

  if (command == "tmcv") {
    char validateFailReason[64] = "OK";
    const bool ok = validateTmcUartForMove(validateFailReason, sizeof(validateFailReason));
    Serial.printf("TMC validate %s reason=%s\n", ok ? "OK" : "FAIL", validateFailReason);
    return;
  }

  if (command == "sgreset") {
    resetStallGuardStats();
    Serial.println("SG stats reset");
    printStallGuardStatus();
    return;
  }

  if (command.startsWith("sgthrs")) {
    uint32_t value = 0;
    if (!parseUnsignedLongCommand(trimmed, "sgthrs", value) || value > 255UL) {
      Serial.println("Usage: sgthrs <0-255>");
      return;
    }
    setStallGuardThreshold(static_cast<uint16_t>(value));
    return;
  }

  if (command.startsWith("tcool")) {
    uint32_t value = 0;
    if (!parseUnsignedLongCommand(trimmed, "tcool", value) || value > 0xFFFFFUL) {
      Serial.println("Usage: tcool <0-1048575>");
      return;
    }
    setStallGuardTcoolThreshold(value);
    return;
  }

  if (command.startsWith("sglog")) {
    uint32_t value = 0;
    if (!parseUnsignedLongCommand(trimmed, "sglog", value) || value > 1UL) {
      Serial.println("Usage: sglog <0|1>");
      return;
    }
    sgLogEnabled = value == 1UL;
    Serial.printf("SG log %s interval_ms=%lu\n", sgLogEnabled ? "ON" : "OFF", sgLogIntervalMs);
    return;
  }

  if (command.startsWith("sgen")) {
    uint32_t value = 0;
    if (!parseUnsignedLongCommand(trimmed, "sgen", value) || value > 1UL) {
      Serial.println("Usage: sgen <0|1>");
      return;
    }
    sgSamplingEnabled = value == 1UL;
    if (!sgSamplingEnabled) {
      resetStallGuardStats();
    }
    Serial.printf("SG sampling %s interval_ms=%lu\n", sgSamplingEnabled ? "ON" : "OFF", sgSampleIntervalMs);
    return;
  }

  if (command.startsWith("sgint")) {
    uint32_t value = 0;
    if (!parseUnsignedLongCommand(trimmed, "sgint", value) || value == 0UL || value > 60000UL) {
      Serial.println("Usage: sgint <ms>, range 1..60000");
      return;
    }
    sgSampleIntervalMs = value;
    Serial.printf("SG sample interval set: %lu ms\n", sgSampleIntervalMs);
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

  Serial.printf("Unknown command '%s'. Use h, 1, 5, b, s, diag, io, angle, as5600, as5600bb, i2cscan, i2cscanbb, i2cpins, motortest, on, off, v <mm/s>, a <mm/s2>, profile trap|direct, i <mA>, mode stealth|spread, microstep 16|8, m <mm> [mm/s], sg, mt, tmcv, sgreset, sgthrs <0-255>, tcool <0-1048575>, sgen <0|1>, sglog <0|1>, or sgint <ms>.\n", trimmed.c_str());
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

bool AppController::parseUnsignedLongCommand(const String& line, const char* prefix, uint32_t& value) const {
  String command = line;
  command.toLowerCase();
  const String prefixText(prefix);
  if (!command.startsWith(prefixText)) {
    return false;
  }

  String args = line.substring(prefixText.length());
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
  if (end == cursor || parsed > 4294967295UL) {
    return false;
  }

  cursor = end;
  while (isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }
  if (*cursor != '\0') {
    return false;
  }

  value = static_cast<uint32_t>(parsed);
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

bool AppController::validateMoveRequest(float distanceMm, float speedMmS) {
  if (!motorPowerEnabled_) {
    printRejectDetail("move_validate", "motor_power_off");
    Serial.println("Move rejected: motor power is OFF. Send 'on' first, then home again.");
    return false;
  }

#if ACTIVE_DRIVER == DRIVER_TMC2209
  if (TMC_VALIDATE_UART_BEFORE_MOVE) {
    char validateFailReason[64] = "OK";
    if (!validateTmcUartForMove(validateFailReason, sizeof(validateFailReason))) {
      if (TMC_BLOCK_MOVE_ON_UART_VALIDATE_FAIL) {
        printRejectDetail("move_validate", validateFailReason);
        Serial.println("Move rejected: TMC2209 UART validation failed");
        return false;
      }
      Serial.printf("Move UART validation warning: %s. Continuing STEP/DIR motion.\n", validateFailReason);
    }
  }
#endif

  if (!isfinite(distanceMm) || !isfinite(speedMmS)) {
    printRejectDetail("move_validate", "non_finite_request");
    Serial.println("Move rejected: distance and speed must be finite numbers");
    printMoveUsage();
    return false;
  }

  if (!validateMoveSpeed(speedMmS)) {
    printRejectDetail("move_validate", "speed_out_of_range");
    printMoveUsage();
    return false;
  }

  if (state_ != State::Ready) {
    printRejectDetail("move_validate", "state_not_ready");
    Serial.printf("Move rejected: state=%s, homed=%s\n", stateName(), axis_.isHomed() ? "true" : "false");
    return false;
  }

  const float targetMm = axis_.currentPositionMm() + distanceMm;
  if (!axis_.isWithinSoftLimit(targetMm)) {
    printRejectDetail("move_validate", "soft_limit");
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
  sgCaptureActive = false;
  motion_.resetTimingStats();
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
    printRejectDetail("current_set", "state_busy");
    printCurrentStatusCsv("REJECTED_STATE");
    Serial.printf("Current rejected: state=%s\n", stateName());
    return;
  }
  if (currentMa < TEST_CURRENT_MIN_MA || currentMa > TEST_CURRENT_MAX_MA) {
    printRejectDetail("current_set", "out_of_range");
    printCurrentStatusCsv("REJECTED_RANGE");
    Serial.printf("Current rejected: %u mA is outside %u..%u mA\n",
                  currentMa,
                  TEST_CURRENT_MIN_MA,
                  TEST_CURRENT_MAX_MA);
    return;
  }

#if ACTIVE_DRIVER == DRIVER_TMC2209
  refreshTmcUartStatus();
  if (!tmcUartOk) {
    printRejectDetail("current_set", "tmc_uart_not_ok");
    printCurrentStatusCsv("REJECTED_UART");
    Serial.println("Current rejected: TMC2209 UART is not OK");
    return;
  }
  tmcRuntimeRmsCurrentMa = currentMa;
  applyTmc2209Current(TmcDriver);
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    delay(50);
    refreshTmcUartStatus();
    if (tmcUartOk) {
      break;
    }
    applyTmc2209Current(TmcDriver);
  }
  Serial.printf("TMC2209 current set: requestedRms=%u mA estimatedRun=%.0f mA estimatedHold=%.0f mA uart=%s\n",
                tmcRuntimeRmsCurrentMa,
                tmcConfiguredRunRmsMa,
                tmcConfiguredHoldRmsMa,
                tmcUartOk ? "OK" : "FAIL");
  printCurrentStatusCsv(tmcUartOk ? "OK" : "UART_FAIL");
#else
  Serial.println("Current command ignored: ACTIVE_DRIVER is not TMC2209");
  printCurrentStatusCsv("NO_TMC2209");
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
  sgCaptureActive = false;
  motion_.resetTimingStats();
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

bool AppController::readStallGuardResult(uint16_t& sgResult) {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  if (!tmcUartOk) {
    return false;
  }
  sgResult = TmcDriver.SG_RESULT();
  return true;
#else
  (void)sgResult;
  return false;
#endif
}

void AppController::updateStallGuardStats() {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  const uint32_t nowMs = millis();
  const bool captureWindowActive = sgCaptureActive
                                   && sgSamplingEnabled
                                   && state_ == State::Moving
                                   && motion_.isMoving()
                                   && nowMs - sgCaptureStartedMs >= SG_IGNORE_AFTER_MOVE_START_MS;

  if (captureWindowActive && nowMs - lastSgUpdateMs >= sgSampleIntervalMs) {
    lastSgUpdateMs = nowMs;

    uint16_t sgResult = 0;
    if (readStallGuardResult(sgResult)) {
      sgCurrent = sgResult;
      sgCurrentValid = true;
      ++sgCount;
      if (sgResult == 0) {
        ++sgZeroCount;
      } else if (!sgStatsHaveValidSample) {
        sgMinValid = sgResult;
        sgMaxValid = sgResult;
        sgStatsHaveValidSample = true;
        sgSumValid += sgResult;
        ++sgValidCount;
        if (sgResult < SG_LOW_TH_50) {
          ++sgLowCount50;
        }
        if (sgResult < SG_LOW_TH_100) {
          ++sgLowCount100;
        }
        if (sgResult < SG_LOW_TH_150) {
          ++sgLowCount150;
        }
      } else {
        if (sgResult < sgMinValid) {
          sgMinValid = sgResult;
        }
        if (sgResult > sgMaxValid) {
          sgMaxValid = sgResult;
        }
        sgSumValid += sgResult;
        ++sgValidCount;
        if (sgResult < SG_LOW_TH_50) {
          ++sgLowCount50;
        }
        if (sgResult < SG_LOW_TH_100) {
          ++sgLowCount100;
        }
        if (sgResult < SG_LOW_TH_150) {
          ++sgLowCount150;
        }
      }
    } else {
      sgCurrentValid = false;
      if (nowMs - lastSgErrorMs >= 1000) {
        lastSgErrorMs = nowMs;
        Serial.println("SG_ERROR,reason=uart_not_ok");
      }
    }
  }

  if (sgLogEnabled && nowMs - lastSgLogMs >= sgLogIntervalMs) {
    lastSgLogMs = nowMs;
    if (!sgStatsHaveValidSample) {
      if (sgCurrentValid) {
        Serial.printf("SGLOG,t_ms=%lu,sg=%u,sg_min=NA,sg_max=NA,sg_avg=NA,sg_count=%lu,sg_min_valid=NA,sg_max_valid=NA,sg_avg_valid=NA,sg_valid_count=%lu,sg_zero_count=%lu,sg_low_count_50=%lu,sg_low_count_100=%lu,sg_low_count_150=%lu,sg_low_ratio_50=NA,sg_low_ratio_100=NA,sg_low_ratio_150=NA,diag=NA\n",
                      nowMs,
                      sgCurrent,
                      sgCount,
                      sgValidCount,
                      sgZeroCount,
                      sgLowCount50,
                      sgLowCount100,
                      sgLowCount150);
      } else {
        Serial.printf("SGLOG,t_ms=%lu,sg=NA,sg_min=NA,sg_max=NA,sg_avg=NA,sg_count=%lu,sg_min_valid=NA,sg_max_valid=NA,sg_avg_valid=NA,sg_valid_count=%lu,sg_zero_count=%lu,sg_low_count_50=%lu,sg_low_count_100=%lu,sg_low_count_150=%lu,sg_low_ratio_50=NA,sg_low_ratio_100=NA,sg_low_ratio_150=NA,diag=NA\n",
                      nowMs,
                      sgCount,
                      sgValidCount,
                      sgZeroCount,
                      sgLowCount50,
                      sgLowCount100,
                      sgLowCount150);
      }
    } else {
      const float sgAvgValid = static_cast<float>(static_cast<double>(sgSumValid) / static_cast<double>(sgValidCount));
      const float sgLowRatio50 = static_cast<float>(static_cast<double>(sgLowCount50) / static_cast<double>(sgValidCount));
      const float sgLowRatio100 = static_cast<float>(static_cast<double>(sgLowCount100) / static_cast<double>(sgValidCount));
      const float sgLowRatio150 = static_cast<float>(static_cast<double>(sgLowCount150) / static_cast<double>(sgValidCount));
      if (sgCurrentValid) {
        Serial.printf("SGLOG,t_ms=%lu,sg=%u,sg_min=%u,sg_max=%u,sg_avg=%.1f,sg_count=%lu,sg_min_valid=%u,sg_max_valid=%u,sg_avg_valid=%.1f,sg_valid_count=%lu,sg_zero_count=%lu,sg_low_count_50=%lu,sg_low_count_100=%lu,sg_low_count_150=%lu,sg_low_ratio_50=%.4f,sg_low_ratio_100=%.4f,sg_low_ratio_150=%.4f,diag=NA\n",
                      nowMs,
                      sgCurrent,
                      sgMinValid,
                      sgMaxValid,
                      sgAvgValid,
                      sgCount,
                      sgMinValid,
                      sgMaxValid,
                      sgAvgValid,
                      sgValidCount,
                      sgZeroCount,
                      sgLowCount50,
                      sgLowCount100,
                      sgLowCount150,
                      sgLowRatio50,
                      sgLowRatio100,
                      sgLowRatio150);
      } else {
        Serial.printf("SGLOG,t_ms=%lu,sg=NA,sg_min=%u,sg_max=%u,sg_avg=%.1f,sg_count=%lu,sg_min_valid=%u,sg_max_valid=%u,sg_avg_valid=%.1f,sg_valid_count=%lu,sg_zero_count=%lu,sg_low_count_50=%lu,sg_low_count_100=%lu,sg_low_count_150=%lu,sg_low_ratio_50=%.4f,sg_low_ratio_100=%.4f,sg_low_ratio_150=%.4f,diag=NA\n",
                      nowMs,
                      sgMinValid,
                      sgMaxValid,
                      sgAvgValid,
                      sgCount,
                      sgMinValid,
                      sgMaxValid,
                      sgAvgValid,
                      sgValidCount,
                      sgZeroCount,
                      sgLowCount50,
                      sgLowCount100,
                      sgLowCount150,
                      sgLowRatio50,
                      sgLowRatio100,
                      sgLowRatio150);
      }
    }
  }
#endif
}

void AppController::resetStallGuardStats() {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  sgCurrentValid = false;
  sgStatsHaveValidSample = false;
  sgCurrent = 0;
  sgMinValid = 0;
  sgMaxValid = 0;
  sgSumValid = 0;
  sgCount = 0;
  sgValidCount = 0;
  sgZeroCount = 0;
  sgLowCount50 = 0;
  sgLowCount100 = 0;
  sgLowCount150 = 0;
  sgCaptureActive = false;
  sgCaptureStartedMs = 0;
  motion_.resetTimingStats();
  moveStartLimitState = "NA";
  moveEndLimitState = "NA";
  timeoutLimitState = "NA";
  limitFirstTriggerTiming = "NONE";
  limitTransitionCount = 0;
  moveLimitStateInitialized = false;
  lastSgUpdateMs = millis();
#endif
}

void AppController::printStallGuardStatus() {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  if (!sgStatsHaveValidSample) {
    if (sgCurrentValid) {
      Serial.printf("SG_RESULT=%u,SG_MIN=NA,SG_MAX=NA,SG_AVG=NA,SG_COUNT=%lu,SG_MIN_VALID=NA,SG_MAX_VALID=NA,SG_AVG_VALID=NA,SG_VALID_COUNT=%lu,SG_ZERO_COUNT=%lu,SGTHRS=%u,TCOOLTHRS=%lu,DIAG=NA,DIAG_TRIGGERED=NA\n",
                    sgCurrent,
                    sgCount,
                    sgValidCount,
                    sgZeroCount,
                    sgThreshold,
                    sgTcoolThreshold);
    } else {
      Serial.printf("SG_RESULT=NA,SG_MIN=NA,SG_MAX=NA,SG_AVG=NA,SG_COUNT=%lu,SG_MIN_VALID=NA,SG_MAX_VALID=NA,SG_AVG_VALID=NA,SG_VALID_COUNT=%lu,SG_ZERO_COUNT=%lu,SGTHRS=%u,TCOOLTHRS=%lu,DIAG=NA,DIAG_TRIGGERED=NA\n",
                    sgCount,
                    sgValidCount,
                    sgZeroCount,
                    sgThreshold,
                    sgTcoolThreshold);
    }
    return;
  }

  const float sgAvgValid = static_cast<float>(static_cast<double>(sgSumValid) / static_cast<double>(sgValidCount));
  if (sgCurrentValid) {
    Serial.printf("SG_RESULT=%u,SG_MIN=%u,SG_MAX=%u,SG_AVG=%.1f,SG_COUNT=%lu,SG_MIN_VALID=%u,SG_MAX_VALID=%u,SG_AVG_VALID=%.1f,SG_VALID_COUNT=%lu,SG_ZERO_COUNT=%lu,SGTHRS=%u,TCOOLTHRS=%lu,DIAG=NA,DIAG_TRIGGERED=NA\n",
                  sgCurrent,
                  sgMinValid,
                  sgMaxValid,
                  sgAvgValid,
                  sgCount,
                  sgMinValid,
                  sgMaxValid,
                  sgAvgValid,
                  sgValidCount,
                  sgZeroCount,
                  sgThreshold,
                  sgTcoolThreshold);
  } else {
    Serial.printf("SG_RESULT=NA,SG_MIN=%u,SG_MAX=%u,SG_AVG=%.1f,SG_COUNT=%lu,SG_MIN_VALID=%u,SG_MAX_VALID=%u,SG_AVG_VALID=%.1f,SG_VALID_COUNT=%lu,SG_ZERO_COUNT=%lu,SGTHRS=%u,TCOOLTHRS=%lu,DIAG=NA,DIAG_TRIGGERED=NA\n",
                  sgMinValid,
                  sgMaxValid,
                  sgAvgValid,
                  sgCount,
                  sgMinValid,
                  sgMaxValid,
                  sgAvgValid,
                  sgValidCount,
                  sgZeroCount,
                  sgThreshold,
                  sgTcoolThreshold);
  }
#else
  Serial.println("SG_RESULT=NA,SG_MIN=NA,SG_MAX=NA,SG_AVG=NA,SG_COUNT=0,SG_MIN_VALID=NA,SG_MAX_VALID=NA,SG_AVG_VALID=NA,SG_VALID_COUNT=0,SG_ZERO_COUNT=0,SGTHRS=NA,TCOOLTHRS=NA,DIAG=NA,DIAG_TRIGGERED=NA");
#endif
}

void AppController::printStallGuardSummary() {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  const uint32_t timingCount = motion_.timingUpdateCount();
  const float timingAvgGapUs = motion_.timingAvgUpdateGapUs();
  const String timingCountText = timingCount == 0 ? "NA" : String(timingCount);
  const String timingMaxGapText = timingCount == 0 ? "NA" : String(motion_.timingMaxUpdateGapUs());
  const String timingAvgGapText = isnan(timingAvgGapUs) ? "NA" : String(timingAvgGapUs, 1);

  if (!sgStatsHaveValidSample) {
    Serial.printf("TEST_SG_SUMMARY,sg_min=NA,sg_max=NA,sg_avg=NA,sg_count=%lu,sg_min_valid=NA,sg_max_valid=NA,sg_avg_valid=NA,sg_valid_count=%lu,sg_zero_count=%lu,sg_low_count_50=%lu,sg_low_count_100=%lu,sg_low_count_150=%lu,sg_low_ratio_50=NA,sg_low_ratio_100=NA,sg_low_ratio_150=NA,motion_update_count=%s,max_update_gap_us=%s,avg_update_gap_us=%s,diag_triggered=NA,sgthrs=%u,tcoolthrs=%lu\n",
                  sgCount,
                  sgValidCount,
                  sgZeroCount,
                  sgLowCount50,
                  sgLowCount100,
                  sgLowCount150,
                  timingCountText.c_str(),
                  timingMaxGapText.c_str(),
                  timingAvgGapText.c_str(),
                  sgThreshold,
                  sgTcoolThreshold);
    return;
  }

  const float sgAvgValid = static_cast<float>(static_cast<double>(sgSumValid) / static_cast<double>(sgValidCount));
  const float sgLowRatio50 = static_cast<float>(static_cast<double>(sgLowCount50) / static_cast<double>(sgValidCount));
  const float sgLowRatio100 = static_cast<float>(static_cast<double>(sgLowCount100) / static_cast<double>(sgValidCount));
  const float sgLowRatio150 = static_cast<float>(static_cast<double>(sgLowCount150) / static_cast<double>(sgValidCount));
  Serial.printf("TEST_SG_SUMMARY,sg_min=%u,sg_max=%u,sg_avg=%.1f,sg_count=%lu,sg_min_valid=%u,sg_max_valid=%u,sg_avg_valid=%.1f,sg_valid_count=%lu,sg_zero_count=%lu,sg_low_count_50=%lu,sg_low_count_100=%lu,sg_low_count_150=%lu,sg_low_ratio_50=%.4f,sg_low_ratio_100=%.4f,sg_low_ratio_150=%.4f,motion_update_count=%s,max_update_gap_us=%s,avg_update_gap_us=%s,diag_triggered=NA,sgthrs=%u,tcoolthrs=%lu\n",
                sgMinValid,
                sgMaxValid,
                sgAvgValid,
                sgCount,
                sgMinValid,
                sgMaxValid,
                sgAvgValid,
                sgValidCount,
                sgZeroCount,
                sgLowCount50,
                sgLowCount100,
                sgLowCount150,
                sgLowRatio50,
                sgLowRatio100,
                sgLowRatio150,
                timingCountText.c_str(),
                timingMaxGapText.c_str(),
                timingAvgGapText.c_str(),
                sgThreshold,
                sgTcoolThreshold);
#else
  Serial.println("TEST_SG_SUMMARY,sg_min=NA,sg_max=NA,sg_avg=NA,sg_count=NA,sg_min_valid=NA,sg_max_valid=NA,sg_avg_valid=NA,sg_valid_count=NA,sg_zero_count=NA,sg_low_count_50=NA,sg_low_count_100=NA,sg_low_count_150=NA,sg_low_ratio_50=NA,sg_low_ratio_100=NA,sg_low_ratio_150=NA,motion_update_count=NA,max_update_gap_us=NA,avg_update_gap_us=NA,diag_triggered=NA,sgthrs=NA,tcoolthrs=NA");
#endif
}

void AppController::printMotionTimingSummary() {
  const uint32_t timingCount = motion_.timingUpdateCount();
  const float timingAvgGapUs = motion_.timingAvgUpdateGapUs();
  const String timingCountText = timingCount == 0 ? "NA" : String(timingCount);
  const String timingMaxGapText = timingCount == 0 ? "NA" : String(motion_.timingMaxUpdateGapUs());
  const String timingAvgGapText = isnan(timingAvgGapUs) ? "NA" : String(timingAvgGapUs, 1);
  const String timeoutPositionText = state_ == State::Moving ? String(axis_.currentPositionMm(), 4) : "NA";
  const String timeoutTargetText = state_ == State::Moving ? String(motion_.targetMm(), 4) : "NA";
  const String timeoutRemainingText = state_ == State::Moving ? String(motion_.remainingSteps()) : "NA";
  Serial.printf("MOTION_TIMING_SUMMARY,motion_update_count=%s,max_update_gap_us=%s,avg_update_gap_us=%s,move_start_limit_state=%s,move_end_limit_state=%s,home_end_limit_state=%s,timeout_limit_state=%s,limit_transition_count=%lu,limit_first_trigger_timing=%s,timeout_current_position=%s,timeout_target_position=%s,timeout_remaining_steps=%s,heartbeat_enabled=%u,heartbeat_pin=%d,last_loop_gap_us=%lu,max_loop_gap_us=%lu,reset_reason=%s\n",
                timingCountText.c_str(),
                timingMaxGapText.c_str(),
                timingAvgGapText.c_str(),
                moveStartLimitState,
                moveEndLimitState,
                homeEndLimitState,
                timeoutLimitState,
                limitTransitionCount,
                limitFirstTriggerTiming,
                timeoutPositionText.c_str(),
                timeoutTargetText.c_str(),
                timeoutRemainingText.c_str(),
                HEARTBEAT_ENABLED ? 1 : 0,
                PIN_HEARTBEAT,
                static_cast<unsigned long>(lastLoopGapUs_),
                static_cast<unsigned long>(maxLoopGapUs_),
                resetReasonName());
}

void AppController::printMoveTiming(uint32_t targetReachedUs, uint32_t completePrintBeforeUs) const {
  const long currentSteps = axis_.currentPositionSteps();
  const long movedSteps = currentSteps - moveStartSteps_;
  const float distanceMm = static_cast<float>(movedSteps) / axis_.stepsPerMm();
  const uint32_t firmwareMotionElapsedUs = targetReachedUs - moveStartedUs_;
  const uint32_t postMotionBeforeCompleteUs = completePrintBeforeUs - targetReachedUs;
  Serial.printf("MOVE_TIMING,delta_mm=%.4f,start_steps=%ld,target_steps=%ld,moved_steps=%ld,distance_mm=%.4f,speed_mm_s=%.4f,accel_mm_s2=%.4f,profile=%s,started_us=%lu,target_reached_us=%lu,complete_print_us=%lu,complete_print_before_us=%lu,firmware_motion_elapsed_ms=%.3f,post_motion_before_complete_ms=%.3f,steps_per_mm=%.4f,microsteps=%u\n",
                moveCommandDeltaMm_,
                moveStartSteps_,
                moveTargetSteps_,
                movedSteps,
                distanceMm,
                moveCommandSpeedMmS_,
                moveCommandAccelMmS2_,
                motion_.profileName(),
                static_cast<unsigned long>(moveStartedUs_),
                static_cast<unsigned long>(targetReachedUs),
                static_cast<unsigned long>(completePrintBeforeUs),
                static_cast<unsigned long>(completePrintBeforeUs),
                static_cast<double>(firmwareMotionElapsedUs) / 1000.0,
                static_cast<double>(postMotionBeforeCompleteUs) / 1000.0,
                axis_.stepsPerMm(),
                runtimeMicrosteps);
}

void AppController::printRejectDetail(const char* stage, const char* reason) {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  const uint16_t currentBeforeRe = tmcRuntimeRmsCurrentMa;
#else
  const uint16_t currentBeforeRe = 0;
#endif
  Serial.printf("RE_DETAIL,re_stage=%s,re_reason=%s,current_before_re=%u,position_before_re=%.4f,limit_state_before_re=%s,motion_state_before_re=%s\n",
                stage,
                reason,
                currentBeforeRe,
                axis_.currentPositionMm(),
                limit_.isPressedDebounced() ? "ON" : "OFF",
                motion_.stateName());
}

void AppController::printCurrentStatusCsv(const char* driverStatus) {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  Serial.printf("CURRENT_STATUS,requested_current_ma=%u,applied_current_ma=%u,config_tmc_uart_ok=%u,tmc_uart_ok=%u,driver_status=%s\n",
                tmcRuntimeRmsCurrentMa,
                TmcDriver.rms_current(),
                tmcUartOk ? 1 : 0,
                tmcUartOk ? 1 : 0,
                driverStatus);
#else
  (void)driverStatus;
  Serial.println("CURRENT_STATUS,requested_current_ma=NA,applied_current_ma=NA,config_tmc_uart_ok=NA,tmc_uart_ok=NA,driver_status=NO_TMC2209");
#endif
}

bool AppController::validateTmcUartForMove(char* failReason, size_t failReasonSize) {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  if (failReasonSize > 0) {
    failReason[0] = '\0';
  }
  const uint8_t ifcntBefore = TmcDriver.IFCNT();
  const uint8_t connectionResult = TmcDriver.test_connection();
  const uint8_t ifcntAfter = TmcDriver.IFCNT();
  const uint8_t ifcntDelta = static_cast<uint8_t>(ifcntAfter - ifcntBefore);
  const bool validateOk = connectionResult == 0;
  tmcUartOk = validateOk;

  const uint8_t gstat = TmcDriver.GSTAT();
  const uint32_t drvStatus = TmcDriver.DRV_STATUS();
  const uint16_t appliedCurrentMa = TmcDriver.rms_current();
  const int32_t currentErrorMa = static_cast<int32_t>(appliedCurrentMa) - static_cast<int32_t>(tmcRuntimeRmsCurrentMa);
  const float currentErrorRatio = tmcRuntimeRmsCurrentMa > 0
                                      ? static_cast<float>(currentErrorMa) / static_cast<float>(tmcRuntimeRmsCurrentMa)
                                      : 0.0F;
  const uint16_t currentToleranceMa = static_cast<uint16_t>(tmcRuntimeRmsCurrentMa / 10) > 50
                                          ? static_cast<uint16_t>(tmcRuntimeRmsCurrentMa / 10)
                                          : 50;
  const bool currentWithinTolerance = labs(currentErrorMa) <= currentToleranceMa;

  const char* detail = "OK";
  if (!validateOk) {
    snprintf(failReason, failReasonSize, "test_connection_%u_ifcnt_delta_%u", connectionResult, ifcntDelta);
    detail = failReason;
  } else if (!currentWithinTolerance) {
    detail = "current_mismatch_nonfatal";
  }

  Serial.printf("VALIDATE_TMC,validate_tmc_uart_ok=%u,validate_driver_status=%s,validate_ifcnt_before=%u,validate_ifcnt_after=%u,validate_ifcnt_delta=%u,validate_gstat=%u,validate_drv_status=%lu,validate_requested_current_ma=%u,validate_applied_current_ma=%u,validate_current_error_ma=%ld,validate_current_error_ratio=%.4f,validate_current_tolerance_ma=%u,validate_fail_reason_detail=%s\n",
                validateOk ? 1 : 0,
                validateOk ? "OK" : "UART_FAIL",
                ifcntBefore,
                ifcntAfter,
                ifcntDelta,
                gstat,
                static_cast<unsigned long>(drvStatus),
                tmcRuntimeRmsCurrentMa,
                appliedCurrentMa,
                static_cast<long>(currentErrorMa),
                currentErrorRatio,
                currentToleranceMa,
                detail);

  if (validateOk && failReasonSize > 0) {
    snprintf(failReason, failReasonSize, "OK");
  }
  return validateOk;
#else
  snprintf(failReason, failReasonSize, "no_tmc2209");
  Serial.println("VALIDATE_TMC,validate_tmc_uart_ok=NA,validate_driver_status=NO_TMC2209,validate_ifcnt_before=NA,validate_ifcnt_after=NA,validate_ifcnt_delta=NA,validate_gstat=NA,validate_drv_status=NA,validate_requested_current_ma=NA,validate_applied_current_ma=NA,validate_current_error_ma=NA,validate_current_error_ratio=NA,validate_current_tolerance_ma=NA,validate_fail_reason_detail=no_tmc2209");
  return false;
#endif
}

bool AppController::setStallGuardThreshold(uint16_t sgthrs) {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  refreshTmcUartStatus();
  if (!tmcUartOk) {
    Serial.println("SGTHRS rejected: TMC2209 UART is not OK");
    return false;
  }
  sgThreshold = static_cast<uint8_t>(sgthrs);
  TmcDriver.SGTHRS(sgThreshold);
  refreshTmcUartStatus();
  Serial.printf("SGTHRS set: %u uart=%s\n", sgThreshold, tmcUartOk ? "OK" : "FAIL");
  printStallGuardStatus();
  return true;
#else
  (void)sgthrs;
  Serial.println("SGTHRS command ignored: ACTIVE_DRIVER is not TMC2209");
  return false;
#endif
}

bool AppController::setStallGuardTcoolThreshold(uint32_t tcoolthrs) {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  refreshTmcUartStatus();
  if (!tmcUartOk) {
    Serial.println("TCOOLTHRS rejected: TMC2209 UART is not OK");
    return false;
  }
  sgTcoolThreshold = tcoolthrs;
  TmcDriver.TCOOLTHRS(sgTcoolThreshold);
  refreshTmcUartStatus();
  Serial.printf("TCOOLTHRS set: %lu uart=%s\n", sgTcoolThreshold, tmcUartOk ? "OK" : "FAIL");
  printStallGuardStatus();
  return true;
#else
  (void)tcoolthrs;
  Serial.println("TCOOLTHRS command ignored: ACTIVE_DRIVER is not TMC2209");
  return false;
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
  sgCaptureActive = false;
  motion_.resetTimingStats();
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

  moveStartedUs_ = micros();
  moveCommandDeltaMm_ = mm;
  moveCommandSpeedMmS_ = speedMmS;
  moveCommandAccelMmS2_ = motion_.accelerationMmS2();
  moveStartSteps_ = axis_.currentPositionSteps();
  const float targetMm = axis_.currentPositionMm() + mm;
  if (!motion_.moveRelativeMm(mm, speedMmS)) {
    sgCaptureActive = false;
    state_ = State::Error;
    Serial.println("Move rejected by motion controller");
    return;
  }
  moveTargetSteps_ = motion_.targetSteps();

  sgCaptureActive = mm > 0.0F;
  sgCaptureStartedMs = millis();
  lastSgUpdateMs = sgCaptureStartedMs;
  motion_.resetTimingStats();
  moveStartLimitState = limit_.isPressedDebounced() ? "ON" : "OFF";
  moveEndLimitState = "NA";
  timeoutLimitState = "NA";
  limitFirstTriggerTiming = limit_.isPressedDebounced() ? "START" : "NONE";
  limitTransitionCount = 0;
  lastMoveLimitState = limit_.isPressedDebounced();
  moveLimitStateInitialized = true;
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
        homeEndLimitState = limit_.isPressedDebounced() ? "ON" : "OFF";
        Serial.println("Homing complete. X=0.00 mm");
        printStatus();
      } else if (homing_.hasError()) {
        state_ = State::Error;
        homeEndLimitState = limit_.isPressedDebounced() ? "ON" : "OFF";
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
      if (moveLimitStateInitialized) {
        const bool currentLimitState = limit_.isPressedDebounced();
        if (currentLimitState != lastMoveLimitState) {
          ++limitTransitionCount;
          if (currentLimitState && strcmp(limitFirstTriggerTiming, "NONE") == 0) {
            limitFirstTriggerTiming = "DURING_MOVE";
          }
          lastMoveLimitState = currentLimitState;
        }
      }
      motion_.update();
      if (!motion_.isMoving() && !motion_.hasError()) {
        const uint32_t targetReachedUs = micros();
        state_ = State::Ready;
        moveEndLimitState = limit_.isPressedDebounced() ? "ON" : "OFF";
        printMotionTimingSummary();
        printStallGuardSummary();
        sgCaptureActive = false;
        const uint32_t completePrintBeforeUs = micros();
        printMoveTiming(targetReachedUs, completePrintBeforeUs);
        Serial.println("Move complete");
        Serial.printf("MOVE_COMPLETE_TIMING,complete_print_before_us=%lu,complete_print_after_us=%lu\n",
                      static_cast<unsigned long>(completePrintBeforeUs),
                      static_cast<unsigned long>(micros()));
        printStatus();
      } else if (motion_.hasError()) {
        state_ = State::Error;
        moveEndLimitState = limit_.isPressedDebounced() ? "ON" : "OFF";
        printMotionTimingSummary();
        printStallGuardSummary();
        Serial.printf("Motion error detail: pos=%.4fmm steps=%ld target=%.4fmm targetSteps=%ld remainingSteps=%ld limitRaw=%s limitDebounced=%s\n",
                      axis_.currentPositionMm(),
                      axis_.currentPositionSteps(),
                      motion_.targetMm(),
                      motion_.targetSteps(),
                      motion_.remainingSteps(),
                      limit_.isPressedRaw() ? "ON" : "OFF",
                      limit_.isPressedDebounced() ? "ON" : "OFF");
        sgCaptureActive = false;
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
  printLoopDiagnostics();
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

void AppController::initAs5600() {
  as5600_.begin();
  Serial.printf("AS5600 I2C initialized: sda=GPIO%u scl=GPIO%u address=0x%02X\n",
                as5600_.sdaPin(),
                as5600_.sclPin(),
                as5600_.address());
}

void AppController::printAs5600BitBangStatus() {
  const As5600Sensor::BitBangReading reading = as5600_.readBitBang();
  Serial.printf("AS5600BB,ok=%u,status_ok=%u,status_ack_mask=0x%02X,status=0x%02X,raw_ok=%u,raw_ack_mask=0x%02X,raw=%u,raw_deg=%.2f,scl=GPIO%u,sda=GPIO%u\n",
                reading.ok ? 1 : 0,
                reading.statusOk ? 1 : 0,
                reading.statusAckMask,
                reading.statusOk ? reading.status : 0,
                reading.rawOk ? 1 : 0,
                reading.rawAckMask,
                reading.rawOk ? reading.rawAngle : 0,
                reading.rawDegrees,
                as5600_.sclPin(),
                as5600_.sdaPin());
}

void AppController::printI2cScan() {
  uint8_t addresses[16] = {};
  Serial.printf("I2C_SCAN,start,mode=wire,sda=GPIO%u,scl=GPIO%u\n", as5600_.sdaPin(), as5600_.sclPin());
  const uint8_t count = as5600_.scanWire(addresses, sizeof(addresses));
  const uint8_t printedCount = count < sizeof(addresses) ? count : sizeof(addresses);
  for (uint8_t index = 0; index < printedCount; ++index) {
    Serial.printf("I2C_SCAN,found=1,address=0x%02X\n", addresses[index]);
  }
  Serial.printf("I2C_SCAN,done,mode=wire,count=%u\n", count);
}

void AppController::printI2cBitBangScan() {
  uint8_t addresses[16] = {};
  Serial.printf("I2C_SCAN,start,mode=bitbang,sda=GPIO%u,scl=GPIO%u\n", as5600_.sdaPin(), as5600_.sclPin());
  const uint8_t count = as5600_.scanBitBang(addresses, sizeof(addresses));
  const uint8_t printedCount = count < sizeof(addresses) ? count : sizeof(addresses);
  for (uint8_t index = 0; index < printedCount; ++index) {
    Serial.printf("I2C_SCAN,found=1,address=0x%02X\n", addresses[index]);
  }
  Serial.printf("I2C_SCAN,done,mode=bitbang,count=%u\n", count);
}

void AppController::printAs5600Angle() {
  const As5600Sensor::Reading reading = as5600_.read();
  Serial.printf("ANGLE,ok=%u,angle_deg=%.2f,angle_raw=%u,raw_deg=%.2f,raw=%u,magnet=%u,weak=%u,strong=%u,magnitude=%u,status=0x%02X\n",
                reading.ok ? 1 : 0,
                reading.angleDegrees,
                reading.angleOk ? reading.angle : 0,
                reading.rawDegrees,
                reading.rawOk ? reading.rawAngle : 0,
                reading.statusOk && As5600Sensor::magnetDetected(reading.status) ? 1 : 0,
                reading.statusOk && As5600Sensor::magnetTooWeak(reading.status) ? 1 : 0,
                reading.statusOk && As5600Sensor::magnetTooStrong(reading.status) ? 1 : 0,
                reading.magnitudeOk ? reading.magnitude : 0,
                reading.statusOk ? reading.status : 0);
}

void AppController::printAs5600Status() {
  const As5600Sensor::Reading reading = as5600_.read();
  Serial.printf("AS5600,ok=%u,sda=GPIO%u,scl=GPIO%u,address=0x%02X,status=0x%02X,magnet_detected=%u,magnet_too_weak=%u,magnet_too_strong=%u,raw=%u,raw_deg=%.2f,angle=%u,angle_deg=%.2f,agc=%u,magnitude=%u\n",
                reading.ok ? 1 : 0,
                as5600_.sdaPin(),
                as5600_.sclPin(),
                as5600_.address(),
                reading.statusOk ? reading.status : 0,
                reading.statusOk && As5600Sensor::magnetDetected(reading.status) ? 1 : 0,
                reading.statusOk && As5600Sensor::magnetTooWeak(reading.status) ? 1 : 0,
                reading.statusOk && As5600Sensor::magnetTooStrong(reading.status) ? 1 : 0,
                reading.rawOk ? reading.rawAngle : 0,
                reading.rawDegrees,
                reading.angleOk ? reading.angle : 0,
                reading.angleDegrees,
                reading.agcOk ? reading.agc : 0,
                reading.magnitudeOk ? reading.magnitude : 0);
}

void AppController::printIoStatus() {
  Serial.printf("IO,limit_pin=GPIO%d,limit_raw=%s,limit_debounced=%s,heartbeat_enabled=%u,heartbeat_pin=%d,as5600_sda=GPIO%u,as5600_scl=GPIO%u,step_pin=GPIO%d,dir_pin=GPIO%d,tmc_rx=GPIO%u,tmc_tx=GPIO%u\n",
                PIN_LIMIT_X_MIN,
                limit_.isPressedRaw() ? "ON" : "OFF",
                limit_.isPressedDebounced() ? "ON" : "OFF",
                HEARTBEAT_ENABLED ? 1 : 0,
                PIN_HEARTBEAT,
                as5600_.sdaPin(),
                as5600_.sclPin(),
                PIN_STEP,
                PIN_DIR,
                TMC_UART_RX_PIN,
                TMC_UART_TX_PIN);
}

void AppController::runI2cPinPulseTest() {
  if (state_ == State::Moving || state_ == State::Homing) {
    Serial.printf("I2C_PIN_TEST,rejected=1,reason=busy,state=%s\n", stateName());
    return;
  }

  const As5600Sensor::PinPulseResult result = as5600_.runPinPulseTest();
  Serial.printf("I2C_PIN_TEST,start,scl=GPIO%u,sda=GPIO%u,scl_initial=%u,sda_initial=%u\n",
                as5600_.sclPin(),
                as5600_.sdaPin(),
                result.sclInitial,
                result.sdaInitial);
  Serial.printf("I2C_PIN_TEST,scl_low_read=%u,sda_read=%u\n",
                result.sclLowRead,
                result.sdaWhileSclLowRead);
  Serial.printf("I2C_PIN_TEST,scl_released_read=%u,sda_read=%u\n",
                result.sclReleasedRead,
                result.sdaAfterSclReleaseRead);
  Serial.printf("I2C_PIN_TEST,scl_read=%u,sda_low_read=%u\n",
                result.sclWhileSdaLowRead,
                result.sdaLowRead);
  Serial.printf("I2C_PIN_TEST,end,scl_final=%u,sda_final=%u\n",
                result.sclFinal,
                result.sdaFinal);
}

void AppController::runMotorStepTest() {
  if (state_ == State::Moving || state_ == State::Homing) {
    Serial.printf("MOTOR_TEST,rejected=1,reason=busy,state=%s\n", stateName());
    return;
  }
  if (!motorPowerEnabled_) {
    Serial.println("MOTOR_TEST,rejected=1,reason=motor_power_off");
    return;
  }

  constexpr uint16_t testSteps = 80;
  constexpr uint32_t stepIntervalUs = 2500;
  const bool limitInitiallyPressed = limit_.isPressedDebounced();
  bool firstDirectionPositive = HOMING_DIRECTION < 0;
  if (!limitInitiallyPressed) {
    firstDirectionPositive = !firstDirectionPositive;
  }

  Serial.printf("MOTOR_TEST,start,steps_each_way=%u,step_interval_us=%lu,first_dir=%s,limit_start=%s\n",
                testSteps,
                static_cast<unsigned long>(stepIntervalUs),
                firstDirectionPositive ? "positive" : "negative",
                limitInitiallyPressed ? "ON" : "OFF");

  for (uint8_t pass = 0; pass < 2; ++pass) {
    const bool directionPositive = pass == 0 ? firstDirectionPositive : !firstDirectionPositive;
    driver_.setDirection(directionPositive);
    delayMicroseconds(10);
    uint16_t completedSteps = 0;

    for (; completedSteps < testSteps; ++completedSteps) {
      const bool movingTowardLimit = (HOMING_DIRECTION < 0 && !directionPositive)
                                     || (HOMING_DIRECTION > 0 && directionPositive);
      if (movingTowardLimit && limit_.isPressedDebounced()) {
        Serial.printf("MOTOR_TEST,pass=%u,stopped=1,reason=limit,steps=%u,dir=%s\n",
                      pass + 1,
                      completedSteps,
                      directionPositive ? "positive" : "negative");
        break;
      }
      driver_.stepPulse();
      delayMicroseconds(stepIntervalUs);
    }

    Serial.printf("MOTOR_TEST,pass=%u,dir=%s,steps=%u,limit=%s\n",
                  pass + 1,
                  directionPositive ? "positive" : "negative",
                  completedSteps,
                  limit_.isPressedDebounced() ? "ON" : "OFF");
    delay(100);
  }

  Serial.printf("MOTOR_TEST,done,step_pulse_count=%lu,limit_end=%s\n",
                static_cast<unsigned long>(driver_.stepPulseCount()),
                limit_.isPressedDebounced() ? "ON" : "OFF");
}

void AppController::printDiagnosticStatus() {
  const char* activeNoStepReason = state_ == State::Homing ? homing_.lastNoStepReasonName() : motion_.lastNoStepReasonName();
  Serial.printf("DIAG,app_state=%s,homing_state=%s,motion_state=%s,current_position_steps=%ld,target_steps=%ld,remaining_steps=%ld,limit_raw=%s,limit_debounced=%s,motion_current_speed_steps_s=%.2f,motion_step_interval_us=%lu,motion_last_step_us=%lu,now_us=%lu,last_step_pulse_us=%lu,step_pulse_count=%lu,last_no_step_reason=%s,motion_no_step_reason=%s,homing_no_step_reason=%s,last_move_reject_reason=%s,heartbeat_enabled=%u,max_loop_gap_us=%lu,max_motion_update_gap_us=%lu\n",
                stateName(),
                homing_.stateName(),
                motion_.stateName(),
                axis_.currentPositionSteps(),
                motion_.targetSteps(),
                motion_.remainingSteps(),
                limit_.isPressedRaw() ? "ON" : "OFF",
                limit_.isPressedDebounced() ? "ON" : "OFF",
                motion_.currentSpeedStepsS(),
                static_cast<unsigned long>(motion_.currentStepIntervalUs()),
                static_cast<unsigned long>(motion_.lastStepUs()),
                static_cast<unsigned long>(micros()),
                static_cast<unsigned long>(driver_.lastStepPulseUs()),
                static_cast<unsigned long>(driver_.stepPulseCount()),
                activeNoStepReason,
                motion_.lastNoStepReasonName(),
                homing_.lastNoStepReasonName(),
                axis_.lastMoveRejectReasonName(),
                HEARTBEAT_ENABLED ? 1 : 0,
                static_cast<unsigned long>(maxLoopGapUs_),
                static_cast<unsigned long>(motion_.timingMaxUpdateGapUs()));
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
