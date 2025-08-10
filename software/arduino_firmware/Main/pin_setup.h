#ifndef PIN_SETUP_H
#define PIN_SETUP_H

// Base joint (Joint 1, X-axis driver)
#define BASE_STEP_PIN     2
#define BASE_DIR_PIN      5
#define BASE_ENABLE_PIN   8

// Shoulder joint (Joint 2, Y-axis driver)
#define SHOULDER_STEP_PIN 3
#define SHOULDER_DIR_PIN  6
#define SHOULDER_ENABLE_PIN 9

// Elbow joint (Joint 3, Z-axis driver)
#define ELBOW_STEP_PIN    4
#define ELBOW_DIR_PIN     7
#define ELBOW_ENABLE_PIN  10

// //Limit switches or endstops
// #define BASE_LIMIT_SWITCH_PIN      11
// #define SHOULDER_LIMIT_SWITCH_PIN  12
// #define ELBOW_LIMIT_SWITCH_PIN     13

// Optional: Additional actuators (servos, gripper, etc.)
// #define GRIPPER_SERVO_PIN  14

void initPins();

#endif // PIN_SETUP_H
