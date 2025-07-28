#include <AccelStepper.h>
#include "stepper_control.h"

// Constants
#define MOTOR_INTERFACE_TYPE 1  // 1 = Driver mode (step + dir)
const float STEPS_PER_REV = 200.0;    // Motor: steps per full revolution
const int MICROSTEPPING = 16;         // Microstepping factor (CNC shield)

// Pin definitions
const int Q1_STEP_PIN = 2;  // Base
const int Q1_DIR_PIN  = 5;

const int Q2_STEP_PIN = 3;  // Shoulder
const int Q2_DIR_PIN  = 6;

const int Q3_STEP_PIN = 4;  // Elbow
const int Q3_DIR_PIN  = 7;

// Stepper objects
AccelStepper stepperQ1(MOTOR_INTERFACE_TYPE, Q1_STEP_PIN, Q1_DIR_PIN);
AccelStepper stepperQ2(MOTOR_INTERFACE_TYPE, Q2_STEP_PIN, Q2_DIR_PIN);
AccelStepper stepperQ3(MOTOR_INTERFACE_TYPE, Q3_STEP_PIN, Q3_DIR_PIN);

// Setup function to initialize steppers
void initSteppers() {
    stepperQ1.setMaxSpeed(1000);
    stepperQ1.setAcceleration(500);

    stepperQ2.setMaxSpeed(1000);
    stepperQ2.setAcceleration(500);

    stepperQ3.setMaxSpeed(1000);
    stepperQ3.setAcceleration(500);
}

//Angle to steps conversion 
long q1ToSteps(float q1) {
    float stepsPerDegree = (STEPS_PER_REV * MICROSTEPPING * Q1_PULLEY_RATIO) / 360.0;
    return (long)(q1 * stepsPerDegree);
}

long q2ToSteps(float q2) {
    float stepsPerDegree = (STEPS_PER_REV * MICROSTEPPING * Q2_PULLEY_RATIO) / 360.0;
    return (long)(q2 * stepsPerDegree);
}

long q3ToSteps(float q3) {
    float stepsPerDegree = (STEPS_PER_REV * MICROSTEPPING * Q3_PULLEY_RATIO) / 360.0;
    return (long)(q3 * stepsPerDegree);
}

// Move motors to specified angles
void moveToAngles(float q1, float q2, float q3) {
    stepperQ1.moveTo(q1ToSteps(q1));
    stepperQ2.moveTo(q2ToSteps(q2));
    stepperQ3.moveTo(q3ToSteps(q3));
}

// Called in main loop to run the steppers
void runSteppers() {
    stepperQ1.run();
    stepperQ2.run();
    stepperQ3.run();
}