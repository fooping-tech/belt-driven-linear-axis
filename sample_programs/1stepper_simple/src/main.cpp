#include <Arduino.h>
#include <AccelStepper.h>
#include <M5Unified.h>

constexpr uint8_t STEP_PIN = 5;
constexpr uint8_t DIR_PIN = 6;

constexpr uint32_t STEP_PULSE_US = 5;
constexpr uint32_t LONG_PRESS_MS = 700;
constexpr uint32_t MODE_PRESS_MS = 2000;
constexpr float ACCELERATION_STEPS_PER_SEC2 = 30.0F;
constexpr long CONTINUOUS_TARGET_STEPS = 1000000L;
constexpr uint32_t SPEEDS_STEPS_PER_SEC[] = {1, 2, 5, 10, 25, 50};
constexpr size_t SPEED_COUNT = sizeof(SPEEDS_STEPS_PER_SEC) / sizeof(SPEEDS_STEPS_PER_SEC[0]);

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

bool directionForward = true;
bool useAccelStepper = true;
size_t speedIndex = 0;
uint32_t lastStepUs = 0;
uint32_t pressStartedMs = 0;
bool wasPressed = false;

float currentSpeedStepsPerSec() {
  return static_cast<float>(SPEEDS_STEPS_PER_SEC[speedIndex]);
}

void updateMotionTarget() {
  stepper.setMaxSpeed(currentSpeedStepsPerSec());
  stepper.moveTo(directionForward ? CONTINUOUS_TARGET_STEPS : -CONTINUOUS_TARGET_STEPS);
}

uint32_t currentStepIntervalUs() {
  return 1000000UL / SPEEDS_STEPS_PER_SEC[speedIndex];
}

void drawStatus() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(2);
  M5.Display.drawString(useAccelStepper ? "MODE: ACCEL" : "MODE: DIRECT",
                        M5.Display.width() / 2,
                        14);

  M5.Display.drawString(directionForward ? "DIR: FWD" : "DIR: REV",
                        M5.Display.width() / 2,
                        42);

  M5.Display.setTextSize(3);
  M5.Display.drawString(String(SPEEDS_STEPS_PER_SEC[speedIndex]) + " step/s",
                        M5.Display.width() / 2,
                        74);

  M5.Display.setTextSize(1);
  M5.Display.drawString("Short DIR Long SPD Hold MODE",
                        M5.Display.width() / 2,
                        112);
}

void toggleDirection() {
  directionForward = !directionForward;
  digitalWrite(DIR_PIN, directionForward ? HIGH : LOW);
  if (useAccelStepper) {
    updateMotionTarget();
  }
  Serial.printf("Direction: %s\n", directionForward ? "forward" : "reverse");
  drawStatus();
}

void nextSpeed() {
  speedIndex = (speedIndex + 1) % SPEED_COUNT;
  if (useAccelStepper) {
    stepper.setMaxSpeed(currentSpeedStepsPerSec());
  }
  Serial.printf("Speed: %lu step/s\n", SPEEDS_STEPS_PER_SEC[speedIndex]);
  drawStatus();
}

void toggleStepperMode() {
  useAccelStepper = !useAccelStepper;
  lastStepUs = micros();

  if (useAccelStepper) {
    stepper.setCurrentPosition(0);
    updateMotionTarget();
  } else {
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, directionForward ? HIGH : LOW);
  }

  Serial.printf("Mode: %s\n", useAccelStepper ? "accel" : "direct");
  drawStatus();
}

void keepContinuousTargetAhead() {
  if (abs(stepper.distanceToGo()) > CONTINUOUS_TARGET_STEPS / 2) {
    return;
  }

  stepper.setCurrentPosition(0);
  updateMotionTarget();
}

void emitOneStepIfDue() {
  const uint32_t nowUs = micros();
  if (nowUs - lastStepUs < currentStepIntervalUs()) {
    return;
  }

  lastStepUs = nowUs;
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(STEP_PIN, LOW);
}

void handleButton() {
  const bool pressed = M5.BtnA.isPressed();
  const uint32_t nowMs = millis();

  if (pressed && !wasPressed) {
    pressStartedMs = nowMs;
  }

  if (!pressed && wasPressed) {
    const uint32_t pressDurationMs = nowMs - pressStartedMs;
    if (pressDurationMs >= MODE_PRESS_MS) {
      toggleStepperMode();
    } else if (pressDurationMs >= LONG_PRESS_MS) {
      nextSpeed();
    } else {
      toggleDirection();
    }
  }

  wasPressed = pressed;
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, directionForward ? HIGH : LOW);

  stepper.setMinPulseWidth(STEP_PULSE_US);
  stepper.setAcceleration(ACCELERATION_STEPS_PER_SEC2);
  updateMotionTarget();

  M5.Display.setRotation(0);
  drawStatus();
  Serial.println("A4988 stepper test ready");
  Serial.printf("Acceleration: %.1f step/s^2\n", ACCELERATION_STEPS_PER_SEC2);
}

void loop() {
  M5.update();
  handleButton();
  if (useAccelStepper) {
    keepContinuousTargetAhead();
    stepper.run();
  } else {
    emitOneStepIfDue();
  }
}
