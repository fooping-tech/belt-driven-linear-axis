#include "AppController.h"

namespace {
const HomingConfig kHomingConfig = {
    HOMING_DIRECTION,
    HOMING_FAST_MM_S,
    HOMING_SLOW_MM_S,
    HOMING_BACKOFF_MM,
    HOMING_MAX_TRAVEL_MM,
    LIMIT_DEBOUNCE_MS,
};
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
    switch (command) {
      case 'h':
      case 'H':
        startHoming();
        break;
      case '1':
        startMoveRelative(10.0F);
        break;
      case '5':
        startMoveRelative(50.0F);
        break;
      case 'b':
      case 'B':
        startMoveRelative(-10.0F);
        break;
      case 's':
      case 'S':
        printStatus();
        break;
      case '\r':
      case '\n':
        break;
      default:
        Serial.printf("Unknown command '%c'. Use h, 1, 5, b, s.\n", command);
        break;
    }
  }
}

void AppController::startHoming() {
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
  if (state_ != State::Ready) {
    Serial.printf("Move rejected: state=%s, homed=%s\n", stateName(), axis_.isHomed() ? "true" : "false");
    return;
  }

  if (!motion_.moveRelativeMm(mm)) {
    state_ = State::Error;
    Serial.printf("Move rejected: target %.2f mm is outside %.2f..%.2f mm or axis is not homed\n",
                  axis_.currentPositionMm() + mm,
                  X_MIN_MM,
                  X_MAX_MM);
    return;
  }

  state_ = State::Moving;
  Serial.printf("Move started: delta=%.2f mm target=%.2f mm\n", mm, motion_.targetMm());
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
  Serial.printf("state=%s homing=%s motion=%s pos=%.2fmm steps=%ld homed=%s limitRaw=%s limitDebounced=%s\n",
                stateName(),
                homing_.stateName(),
                motion_.stateName(),
                axis_.currentPositionMm(),
                axis_.currentPositionSteps(),
                axis_.isHomed() ? "true" : "false",
                limit_.isPressedRaw() ? "ON" : "OFF",
                limit_.isPressedDebounced() ? "ON" : "OFF");
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
  M5.Display.drawString(String("Limit: ") + (limit_.isPressedDebounced() ? "ON" : "OFF"), M5.Display.width() / 2, 82);

  const char* footer = "Long: home  Short: +10";
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
