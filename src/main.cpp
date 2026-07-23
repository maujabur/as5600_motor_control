#include <Arduino.h>

#include "MotorControlApplication.h"

MotorControlApplication application;

void setup() {
  application.setup();
}

void loop() {
  application.update(millis());
}
