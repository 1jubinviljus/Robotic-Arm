#include "stepper_control.h"
#include "pin_setup.h"

void setup() {

  // Initialize and setup all pins on the board
  initPins();

  Serial.begin(9600);
  Serial.println("Setup started...");
}

void loop() {
  moveSteppers();
}
