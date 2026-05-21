#include <Arduino.h>
#include <M5Unified.h>

constexpr uint8_t STEP_PIN = 5;
constexpr uint8_t DIR_PIN = 6;

constexpr uint32_t STEP_PULSE_US = 5;
constexpr uint32_t LONG_PRESS_MS = 700;
constexpr uint32_t SPEEDS_STEPS_PER_SEC[] = {1, 2, 5, 10, 25, 50};
constexpr size_t SPEED_COUNT = sizeof(SPEEDS_STEPS_PER_SEC) / sizeof(SPEEDS_STEPS_PER_SEC[0]);

bool directionForward = true;
size_t speedIndex = 0;
uint32_t lastStepUs = 0;
uint32_t pressStartedMs = 0;
bool wasPressed = false;

uint32_t currentStepIntervalUs() {
  return 1000000UL / SPEEDS_STEPS_PER_SEC[speedIndex];
}

void drawStatus() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(2);
  M5.Display.drawString(directionForward ? "DIR: FWD" : "DIR: REV",
                        M5.Display.width() / 2,
                        22);

  M5.Display.setTextSize(3);
  M5.Display.drawString(String(SPEEDS_STEPS_PER_SEC[speedIndex]) + " step/s",
                        M5.Display.width() / 2,
                        58);

  M5.Display.setTextSize(1);
  M5.Display.drawString("Short: DIR  Long: SPEED",
                        M5.Display.width() / 2,
                        104);
}

void toggleDirection() {
  directionForward = !directionForward;
  digitalWrite(DIR_PIN, directionForward ? HIGH : LOW);
  Serial.printf("Direction: %s\n", directionForward ? "forward" : "reverse");
  drawStatus();
}

void nextSpeed() {
  speedIndex = (speedIndex + 1) % SPEED_COUNT;
  Serial.printf("Speed: %lu step/s\n", SPEEDS_STEPS_PER_SEC[speedIndex]);
  drawStatus();
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
    if (nowMs - pressStartedMs >= LONG_PRESS_MS) {
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

  M5.Display.setRotation(0);
  drawStatus();
  Serial.println("A4988 stepper test ready");
}

void loop() {
  M5.update();
  handleButton();
  emitOneStepIfDue();
}
