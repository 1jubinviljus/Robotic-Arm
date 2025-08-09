#include "stepper_control.h"
#include "pin_setup.h"

void setup() {

  # Initialize and setup all pins on the board
  initPins();

  Serial.begin(9600);
  Serial.println("Setup started...");
}

void loop() {
  for (int i = 0; i < 200; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
}
