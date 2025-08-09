#include "pin_setup.h"
#include "Arduino.h"


void initPins() {
    // Base joint (Joint 1, X-axis driver)
    pinMode(BASE_STEP_PIN, OUTPUT);
    pinMode(BASE_DIR_PIN, OUTPUT);
    pinMode(BASE_ENABLE_PIN, OUTPUT);
    digitalWrite(BASE_ENABLE_PIN, LOW); // Enable driver
    digitalWrite(BASE_DIR_PIN, HIGH); 

    // Shoulder joint (Joint 2, Y-axis driver)
    pinMode(SHOULDER_STEP_PIN, OUTPUT);
    pinMode(SHOULDER_DIR_PIN, OUTPUT);
    pinMode(SHOULDER_ENABLE_PIN, OUTPUT);
    digitalWrite(SHOULDER_ENABLE_PIN, LOW);
    digitalWrite(SHOULDER_DIR_PIN, HIGH); 

    // Elbow joint (Joint 3, Z-axis driver)
    pinMode(ELBOW_STEP_PIN, OUTPUT);
    pinMode(ELBOW_DIR_PIN, OUTPUT);
    pinMode(ELBOW_ENABLE_PIN, OUTPUT);
    digitalWrite(ELBOW_ENABLE_PIN, LOW);
    digitalWrite(ELBOW_DIR_PIN, HIGH); 

//     // Optional: Limit switches or endstops
//     pinMode(BASE_LIMIT_SWITCH_PIN, INPUT_PULLUP);
//     pinMode(SHOULDER_LIMIT_SWITCH_PIN, INPUT_PULLUP);
//     pinMode(ELBOW_LIMIT_SWITCH_PIN, INPUT_PULLUP);
}