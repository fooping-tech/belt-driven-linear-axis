#include <Arduino.h>
#include <AccelStepper.h>
#include <M5Unified.h>

#include "config.h"

#if ACTIVE_DRIVER == DRIVER_TMC2209
#include <TMCStepper.h>
#endif

constexpr uint32_t UART_TEST_INTERVAL_MS = 1000;
constexpr uint32_t GPIO8_TEST_INTERVAL_MS = 500;
constexpr long CONTINUOUS_TARGET_STEPS = 1000000L;
constexpr float SPEEDS_MM_PER_SEC[] = {1.0F, 5.0F, 10.0F, 30.0F, 50.0F};
constexpr size_t SPEED_COUNT = sizeof(SPEEDS_MM_PER_SEC) / sizeof(SPEEDS_MM_PER_SEC[0]);

enum class MotionState : uint8_t {
  Idle,
  Homing,
  OriginSet,
  MovingToTestDistance,
  Complete,
  LimitHeld,
};

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

#if ACTIVE_DRIVER == DRIVER_TMC2209
HardwareSerial TmcSerial(1);
TMC2209Stepper driver(&TmcSerial, R_SENSE, DRIVER_ADDRESS);
bool tmcUartOk = false;
uint32_t lastUartTestMs = 0;
#endif

bool directionForward = true;
size_t speedIndex = 0;
bool wasPressed = false;
bool wasLimitActive = false;
MotionState motionState = MotionState::Idle;
bool originRecorded = false;
bool moveCompleteLogged = false;
#if defined(DIAG_GPIO8_TOGGLE)
bool gpio8TestState = false;
uint32_t lastGpio8TestToggleMs = 0;
#endif

float currentSpeedMmPerSec() {
  return SPEEDS_MM_PER_SEC[speedIndex];
}

float currentSpeedStepsPerSec() {
  return currentSpeedMmPerSec() * STEPS_PER_MM;
}

long mmToSteps(const float mm) {
  return lroundf(mm * STEPS_PER_MM);
}

const char *driverName() {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  return "TMC2209";
#else
  return "A4988";
#endif
}

const char *motionStateName() {
  switch (motionState) {
    case MotionState::Idle:
      return "IDLE";
    case MotionState::Homing:
      return "HOME";
    case MotionState::OriginSet:
      return "ORIGIN";
    case MotionState::MovingToTestDistance:
      return "MOVE";
    case MotionState::Complete:
      return "DONE";
    case MotionState::LimitHeld:
      return "LIMIT";
  }

  return "UNKNOWN";
}

bool limitSwitchActive() {
  // GPIO39 is pulled up internally. Switch ON means it is touching the end stop
  // and pulls the pin to GND.
  return digitalRead(LIMIT_SWITCH_PIN) == LOW;
}

void applyMotionSpeed() {
  stepper.setMaxSpeed(currentSpeedStepsPerSec());
  stepper.setAcceleration(ACCELERATION_MM_PER_SEC2 * STEPS_PER_MM);
}

void drawStatus() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(middle_center);

  M5.Display.setTextSize(2);
  M5.Display.drawString(driverName(), M5.Display.width() / 2, 12);

  M5.Display.setTextSize(1);
  M5.Display.drawString(String("STATE: ") + motionStateName(), M5.Display.width() / 2, 34);
  M5.Display.drawString(String("POS: ") + String(stepper.currentPosition() / STEPS_PER_MM, 1) + "mm",
                        M5.Display.width() / 2,
                        50);
  M5.Display.drawString(String("uSTEP: 1/") + MICROSTEPS, M5.Display.width() / 2, 66);

  M5.Display.setTextSize(2);
  M5.Display.drawString(String(TEST_TRAVEL_MM, 0) + " mm", M5.Display.width() / 2, 88);

  M5.Display.setTextSize(1);
  const char *footer = "Btn: restart homing";
  if (motionState == MotionState::OriginSet) {
    footer = "Btn: move test";
  }
  if (limitSwitchActive()) {
    footer = "LIMIT SWITCH ON";
#if ACTIVE_DRIVER == DRIVER_TMC2209
  } else if (!tmcUartOk) {
    footer = "TMC UART FAIL";
#endif
  }
  M5.Display.drawString(footer, M5.Display.width() / 2, 116);
}

void printStatus() {
  Serial.printf("Driver: %s, state: %s, position: %.2f mm, speed: %.1f mm/s (%.1f step/s), microsteps: %u\n",
                driverName(),
                motionStateName(),
                stepper.currentPosition() / STEPS_PER_MM,
                currentSpeedMmPerSec(),
                currentSpeedStepsPerSec(),
                MICROSTEPS);
}

void startHoming() {
  applyMotionSpeed();
  stepper.setMaxSpeed(HOMING_SPEED_MM_PER_SEC * STEPS_PER_MM);
  motionState = MotionState::Homing;
  originRecorded = false;
  moveCompleteLogged = false;
  directionForward = false;
  digitalWrite(DIR_PIN, HOMING_DIR_LEVEL);
  stepper.moveTo(-CONTINUOUS_TARGET_STEPS);
  Serial.printf("Homing started: moving reverse at %.1f mm/s until GPIO%u goes LOW\n",
                HOMING_SPEED_MM_PER_SEC,
                LIMIT_SWITCH_PIN);
  drawStatus();
}

void recordOriginAndStop() {
  stepper.setCurrentPosition(0);
  stepper.moveTo(0);
  originRecorded = true;
  moveCompleteLogged = false;
  motionState = MotionState::OriginSet;
  Serial.println("Origin recorded, motor stopped");
  drawStatus();
}

void startTestMoveFromOrigin() {
  applyMotionSpeed();
  stepper.setCurrentPosition(0);
  stepper.moveTo(mmToSteps(TEST_TRAVEL_MM));
  directionForward = true;
  digitalWrite(DIR_PIN, TEST_MOVE_DIR_LEVEL);
  originRecorded = true;
  motionState = MotionState::MovingToTestDistance;
  Serial.printf("Origin recorded at limit switch. Moving forward to %.2f mm (%ld steps).\n",
                TEST_TRAVEL_MM,
                mmToSteps(TEST_TRAVEL_MM));
  drawStatus();
}

void handleButton() {
  const bool pressed = M5.BtnA.isPressed();

  if (!pressed && wasPressed) {
    switch (motionState) {
      case MotionState::Idle:
      case MotionState::Complete:
      case MotionState::LimitHeld:
        startHoming();
        break;
      case MotionState::OriginSet:
        startTestMoveFromOrigin();
        break;
      case MotionState::Homing:
      case MotionState::MovingToTestDistance:
        Serial.printf("Button ignored while state=%s\n", motionStateName());
        break;
    }
  }

  wasPressed = pressed;
}

void initStepDirPins() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, directionForward ? DIR_FORWARD_LEVEL : DIR_REVERSE_LEVEL);
  stepper.setPinsInverted(INVERT_DIR_PIN, INVERT_STEP_PIN, INVERT_ENABLE_PIN);
  Serial.printf("STEP/DIR pins: STEP=GPIO%u DIR=GPIO%u dirInverted=%s\n",
                STEP_PIN,
                DIR_PIN,
                INVERT_DIR_PIN ? "true" : "false");
}

void initLimitSwitch() {
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  Serial.printf("Limit switch: GPIO%u INPUT_PULLUP, ON/end-stop = LOW\n", LIMIT_SWITCH_PIN);
}

#if ACTIVE_DRIVER == DRIVER_TMC2209
void testTmcUart(const char *context) {
  const uint8_t connectionResult = driver.test_connection();
  tmcUartOk = connectionResult == 0;
  Serial.printf("TMC2209 UART connection %s: %s (test_connection=%u)\n",
                context,
                tmcUartOk ? "OK" : "FAIL",
                connectionResult);
}

void applyTmc2209Config() {
  driver.GSTAT(TMC_GSTAT_CLEAR);

  driver.I_scale_analog(TMC_I_SCALE_ANALOG);
  driver.internal_Rsense(TMC_INTERNAL_RSENSE);
  driver.en_spreadCycle(TMC_EN_SPREADCYCLE);
  driver.shaft(TMC_SHAFT);
  driver.index_otpw(TMC_INDEX_OTPW);
  driver.index_step(TMC_INDEX_STEP);
  driver.pdn_disable(TMC_PDN_DISABLE);
  driver.mstep_reg_select(TMC_MSTEP_REG_SELECT);
  driver.multistep_filt(TMC_MULTISTEP_FILT);

  driver.senddelay(TMC_SENDDELAY);

  if (TMC_ENABLE_OTP_PROG) {
    driver.OTP_PROG(TMC_OTP_PROG);
  }

  if (TMC_WRITE_FACTORY_CONF) {
    driver.fclktrim(TMC_FCLKTRIM);
    driver.ottrim(TMC_OTTRIM);
  }

  driver.hold_multiplier(HOLD_MULTIPLIER);
  if (TMC_USE_IHOLD_IRUN_DIRECT) {
    driver.ihold(TMC_IHOLD);
    driver.irun(TMC_IRUN);
    driver.iholddelay(TMC_IHOLDDELAY);
  } else {
    driver.rms_current(RMS_CURRENT_MA);
  }
  driver.TPOWERDOWN(TMC_TPOWERDOWN);
  driver.TPWMTHRS(TMC_TPWMTHRS);

  driver.VACTUAL(TMC_VACTUAL);

  driver.toff(TMC_TOFF);
  driver.hstrt(TMC_HSTRT);
  driver.hend(TMC_HEND);
  driver.tbl(TMC_TBL);
  driver.vsense(TMC_VSENSE);
  if (TMC_USE_MRES_DIRECT) {
    driver.mres(TMC_MRES);
  } else {
    driver.microsteps(MICROSTEPS);
  }
  driver.intpol(TMC_INTPOL);
  driver.dedge(TMC_DEDGE);
  driver.diss2g(TMC_DISS2G);
  driver.diss2vs(TMC_DISS2VS);

  driver.pwm_ofs(TMC_PWM_OFS);
  driver.pwm_grad(TMC_PWM_GRAD);
  driver.pwm_freq(TMC_PWM_FREQ);
  driver.pwm_autoscale(TMC_PWM_AUTOSCALE);
  driver.pwm_autograd(TMC_PWM_AUTOGRAD);
  driver.freewheel(TMC_FREEWHEEL);
  driver.pwm_reg(TMC_PWM_REG);
  driver.pwm_lim(TMC_PWM_LIM);

  driver.TCOOLTHRS(TMC_TCOOLTHRS);
  driver.SGTHRS(TMC_SGTHRS);
  driver.semin(TMC_SEMIN);
  driver.seup(TMC_SEUP);
  driver.semax(TMC_SEMAX);
  driver.sedn(TMC_SEDN);
  driver.seimin(TMC_SEIMIN);
}

void printTmc2209Config() {
  Serial.printf("TMC2209 current: rms=%u mA hold_multiplier=%.2f direct_current=%s ihold=%u irun=%u iholddelay=%u\n",
                RMS_CURRENT_MA,
                HOLD_MULTIPLIER,
                TMC_USE_IHOLD_IRUN_DIRECT ? "true" : "false",
                TMC_IHOLD,
                TMC_IRUN,
                TMC_IHOLDDELAY);
  Serial.printf("TMC2209 GCONF: spreadCycle=%s shaft=%s pdn_disable=%s mstep_reg_select=%s multistep_filt=%s\n",
                TMC_EN_SPREADCYCLE ? "true" : "false",
                TMC_SHAFT ? "true" : "false",
                TMC_PDN_DISABLE ? "true" : "false",
                TMC_MSTEP_REG_SELECT ? "true" : "false",
                TMC_MULTISTEP_FILT ? "true" : "false");
  Serial.printf("TMC2209 CHOPCONF: toff=%u hstrt=%u hend=%u tbl=%u vsense=%s microsteps=%u intpol=%s dedge=%s\n",
                TMC_TOFF,
                TMC_HSTRT,
                TMC_HEND,
                TMC_TBL,
                TMC_VSENSE ? "true" : "false",
                MICROSTEPS,
                TMC_INTPOL ? "true" : "false",
                TMC_DEDGE ? "true" : "false");
  Serial.printf("TMC2209 PWMCONF: ofs=%u grad=%u freq=%u autoscale=%s autograd=%s freewheel=%u reg=%u lim=%u\n",
                TMC_PWM_OFS,
                TMC_PWM_GRAD,
                TMC_PWM_FREQ,
                TMC_PWM_AUTOSCALE ? "true" : "false",
                TMC_PWM_AUTOGRAD ? "true" : "false",
                TMC_FREEWHEEL,
                TMC_PWM_REG,
                TMC_PWM_LIM);
  Serial.printf("TMC2209 CoolStep/StallGuard: tcoolthrs=%lu sgthrs=%u semin=%u seup=%u semax=%u sedn=%u seimin=%s\n",
                TMC_TCOOLTHRS,
                TMC_SGTHRS,
                TMC_SEMIN,
                TMC_SEUP,
                TMC_SEMAX,
                TMC_SEDN,
                TMC_SEIMIN ? "true" : "false");
}
#endif

void initDriver() {
#if ACTIVE_DRIVER == DRIVER_TMC2209
  Serial.println("Initializing TMC2209 over UART");
  TmcSerial.begin(TMC_UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  delay(20);

  driver.begin();
  applyTmc2209Config();

  Serial.printf("TMC2209 UART: RX=GPIO%u TX=GPIO%u baud=%lu addr=0b%02u R_SENSE=%.2f\n",
                UART_RX_PIN,
                UART_TX_PIN,
                TMC_UART_BAUDRATE,
                DRIVER_ADDRESS,
                R_SENSE);
  testTmcUart("startup");
  printTmc2209Config();
  Serial.println("If the motor or driver becomes hot, lower RMS_CURRENT_MA in include/config.h.");
#else
  Serial.println("A4988 mode: UART initialization skipped.");
#endif
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  delay(100);

#if defined(DIAG_GPIO8_TOGGLE)
  pinMode(UART_TX_PIN, OUTPUT);
  digitalWrite(UART_TX_PIN, LOW);
  M5.Display.setRotation(0);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(2);
  M5.Display.drawString("GPIO8 TEST", M5.Display.width() / 2, 42);
  M5.Display.setTextSize(1);
  M5.Display.drawString("G8 toggles every 500ms", M5.Display.width() / 2, 78);
  Serial.printf("GPIO8 toggle test: GPIO%u toggles every %lu ms\n",
                UART_TX_PIN,
                GPIO8_TEST_INTERVAL_MS);
  return;
#endif

  initStepDirPins();
  initLimitSwitch();
  initDriver();

  stepper.setMinPulseWidth(STEP_PULSE_US);
  applyMotionSpeed();

  M5.Display.setRotation(0);
  drawStatus();

  Serial.println("STEP/DIR homing and distance move test ready");
  Serial.printf("GT2 20T: %.1f steps/mm, move speed %.1f mm/s, homing speed %.1f mm/s, acceleration %.1f mm/s^2\n",
                STEPS_PER_MM,
                currentSpeedMmPerSec(),
                HOMING_SPEED_MM_PER_SEC,
                ACCELERATION_MM_PER_SEC2);
  Serial.printf("Test sequence: home on GPIO%u LOW, record origin, then move %.2f mm.\n",
                LIMIT_SWITCH_PIN,
                TEST_TRAVEL_MM);
  printStatus();
  startHoming();
}

void loop() {
#if defined(DIAG_GPIO8_TOGGLE)
  const uint32_t gpio8TestNowMs = millis();
  if (gpio8TestNowMs - lastGpio8TestToggleMs >= GPIO8_TEST_INTERVAL_MS) {
    lastGpio8TestToggleMs = gpio8TestNowMs;
    gpio8TestState = !gpio8TestState;
    digitalWrite(UART_TX_PIN, gpio8TestState ? HIGH : LOW);
    Serial.printf("GPIO%u=%s\n", UART_TX_PIN, gpio8TestState ? "HIGH" : "LOW");
  }
  delay(1);
  return;
#endif

  M5.update();
  handleButton();

  const bool limitActive = limitSwitchActive();
  if (limitActive && !wasLimitActive) {
    Serial.println("Limit switch ON");
    wasLimitActive = true;
  } else if (!limitActive && wasLimitActive) {
    Serial.println("Limit switch released");
    wasLimitActive = false;
  }

  if (motionState == MotionState::Homing && limitActive) {
    recordOriginAndStop();
  } else if (motionState == MotionState::MovingToTestDistance && stepper.distanceToGo() == 0) {
    motionState = MotionState::Complete;
    moveCompleteLogged = true;
    Serial.printf("Move complete: position %.2f mm (%ld steps), originRecorded=%s\n",
                  stepper.currentPosition() / STEPS_PER_MM,
                  stepper.currentPosition(),
                  originRecorded ? "true" : "false");
    drawStatus();
  }

#if ACTIVE_DRIVER == DRIVER_TMC2209
  const uint32_t nowMs = millis();
  if (nowMs - lastUartTestMs >= UART_TEST_INTERVAL_MS) {
    lastUartTestMs = nowMs;
    testTmcUart("periodic");
    drawStatus();
  }
#endif

  stepper.run();
}
