#include "stepper_control.h"

void setup() {
  initSteppers();
}

void loop() {
  // Example: move to angles (degrees)
  moveToAngles(45, 30, 15);

  // Keep motors running toward target positions
  runSteppers();
}
