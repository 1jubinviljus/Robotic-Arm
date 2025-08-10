#include "Arduino.h"
#include "stepper_control.h"
#include "pin_setup.h"

void moveSteppers(){
    for (int i = 0; i < 200; i++) {
        digitalWrite(BASE_STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(BASE_STEP_PIN, LOW);
        delayMicroseconds(500);
    }
    delay(1000);
}

