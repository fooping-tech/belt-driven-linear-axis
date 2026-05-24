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

#if ACTIVE_DRIVER == DRIVER_TMC2209
HardwareSerial TmcSerial(1);
TMC2209Stepper TmcDriver(&TmcSerial, TMC_R_SENSE, TMC_DRIVER_ADDRESS);
bool tmcUartOk = false;
uint8_t tmcUartAddress = TMC_DRIVER_ADDRESS;

uint8_t refreshTmcUartStatus() {
  const uint8_t connectionResult = TmcDriver.test_connection();
  tmcUartOk = connectionResult == 0;
  return connectionResult;
}

void applyTmc2209Config(TMC2209Stepper& driver) {
  driver.begin();
  driver.GSTAT(0b111);

  driver.pdn_disable(true);
  driver.mstep_reg_select(true);
  driver.multistep_filt(true);

  driver.rms_current(TMC_RMS_CURRENT_MA);
  driver.hold_multiplier(TMC_HOLD_MULTIPLIER);

  driver.toff(5);
  driver.hstrt(5);
  driver.hend(0);
  driver.tbl(2);
  driver.microsteps(MICROSTEPS);
  driver.intpol(true);

  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(false);
}
#endif
}

AppController::AppController()
    : driver_(PIN_STEP, PIN_DIR, PIN_EN, STEP_PULSE_US, DIR_INVERTED),
      limit_(PIN_LIMIT_X_MIN, true, true, LIMIT_DEBOUNCE_MS),
      axis_(driver_, &limit_, STEPS_PER_MM, HOMING_DIRECTION),
      homing_(axis_, kHomingConfig),
      motion_(axis_, DEFAULT_MOVE_SPEED_MM_S) {}

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
          printStatus();
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
  applyTmc2209Config(TmcDriver);
  const uint8_t connectionResult = TmcDriver.test_connection();
  tmcUartOk = connectionResult == 0;

  Serial.printf("TMC2209 UART connection: %s (test_connection=%u)\n",
                tmcUartOk ? "OK" : "FAIL",
                connectionResult);
  Serial.printf("TMC2209 microsteps set to 1/%u, steps/mm=%.2f, rms_current=%u mA\n",
                MICROSTEPS,
                STEPS_PER_MM,
                TMC_RMS_CURRENT_MA);
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

  Serial.printf("Unknown command '%s'. Use h, 1, 5, b, s, or m <mm> <mm/s>.\n", trimmed.c_str());
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

bool AppController::validateMoveRequest(float distanceMm, float speedMmS) const {
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

  if (speedMmS < TEST_MOVE_MIN_SPEED_MM_S || speedMmS > TEST_MOVE_MAX_SPEED_MM_S) {
    Serial.printf("Move rejected: speed %.2f mm/s is outside %.2f..%.2f mm/s\n",
                  speedMmS,
                  TEST_MOVE_MIN_SPEED_MM_S,
                  TEST_MOVE_MAX_SPEED_MM_S);
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
  Serial.printf("Usage: m <distance_mm> <speed_mm_s>, speed range %.2f..%.2f mm/s\n",
                TEST_MOVE_MIN_SPEED_MM_S,
                TEST_MOVE_MAX_SPEED_MM_S);
}

void AppController::startHoming() {
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
  startMoveRelative(mm, DEFAULT_MOVE_SPEED_MM_S);
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
  Serial.printf("Move started: delta=%.2f mm speed=%.2f mm/s target=%.2f mm softLimit=%.2f..%.2f mm\n",
                mm,
                speedMmS,
                targetMm,
                X_MIN_MM,
                X_MAX_MM);
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

  Serial.printf("state=%s homing=%s motion=%s pos=%.2fmm steps=%ld speed=%.2fmm/s homed=%s limitRaw=%s limitDebounced=%s\n",
                stateName(),
                homing_.stateName(),
                motion_.stateName(),
                axis_.currentPositionMm(),
                axis_.currentPositionSteps(),
                motion_.speedMmS(),
                axis_.isHomed() ? "true" : "false",
                limit_.isPressedRaw() ? "ON" : "OFF",
                limit_.isPressedDebounced() ? "ON" : "OFF");
#if ACTIVE_DRIVER == DRIVER_TMC2209
  Serial.printf("tmc2209_uart=%s test_connection=%u microsteps=1/%u stepsPerMm=%.2f rmsCurrent=%u mA\n",
                tmcUartOk ? "OK" : "FAIL",
                connectionResult,
                MICROSTEPS,
                STEPS_PER_MM,
                TMC_RMS_CURRENT_MA);
  Serial.printf("tmc2209_uart_address=0b%02u rx=GPIO%u tx=GPIO%u\n",
                tmcUartAddress,
                TMC_UART_RX_PIN,
                TMC_UART_TX_PIN);
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
  M5.Display.drawString(String("Speed: ") + String(motion_.speedMmS(), 1) + " mm/s", M5.Display.width() / 2, 82);
  M5.Display.drawString(String("Limit: ") + (limit_.isPressedDebounced() ? "ON" : "OFF"), M5.Display.width() / 2, 98);

  const char* footer = "Serial: m mm speed";
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
