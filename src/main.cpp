#include <Arduino.h>

#include "AppController.h"

AppController app;

void setup() {
  app.begin();
}

void loop() {
  app.update();
}
