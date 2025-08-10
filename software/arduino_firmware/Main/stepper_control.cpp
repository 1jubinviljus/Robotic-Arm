#include "Arduino.h"
#include "stepper_control.h"
#include "robot_parameters.h"
#include "pin_setup.h"

// Define step variables here (only once)
long baseTargetSteps = 0;
long shoulderTargetSteps = 0;
long elbowTargetSteps = 0;
long baseCurrentSteps = 0;
long shoulderCurrentSteps = 0;
long elbowCurrentSteps = 0;


// Move each joint to the specified angle (degrees)
void moveSteppers(long baseAngle, long shoulderAngle, long elbowAngle) {
    // Calculate required steps for each joint (using steps per degree)
    baseTargetSteps = (long)(baseAngle * baseStepsPerDegree);
    shoulderTargetSteps = (long)(shoulderAngle * shoulderStepsPerDegree);
    elbowTargetSteps = (long)(elbowAngle * elbowStepsPerDegree);

    // Move Base Joint
    int baseDir = (baseTargetSteps >= baseCurrentSteps) ? HIGH : LOW;
    digitalWrite(BASE_DIR_PIN, baseDir);
    long baseStepsToMove = abs(baseTargetSteps - baseCurrentSteps);
    for (long i = 0; i < baseStepsToMove; i++) {
        digitalWrite(BASE_STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(BASE_STEP_PIN, LOW);
        delayMicroseconds(500);
    }
    baseCurrentSteps = baseTargetSteps;

    // Move Shoulder Joint
    int shoulderDir = (shoulderTargetSteps >= shoulderCurrentSteps) ? HIGH : LOW;
    digitalWrite(SHOULDER_DIR_PIN, shoulderDir);
    long shoulderStepsToMove = abs(shoulderTargetSteps - shoulderCurrentSteps);
    for (long i = 0; i < shoulderStepsToMove; i++) {
        digitalWrite(SHOULDER_STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(SHOULDER_STEP_PIN, LOW);
        delayMicroseconds(500);
    }
    shoulderCurrentSteps = shoulderTargetSteps;

    // Move Elbow Joint
    int elbowDir = (elbowTargetSteps >= elbowCurrentSteps) ? HIGH : LOW;
    digitalWrite(ELBOW_DIR_PIN, elbowDir);
    long elbowStepsToMove = abs(elbowTargetSteps - elbowCurrentSteps);
    for (long i = 0; i < elbowStepsToMove; i++) {
        digitalWrite(ELBOW_STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(ELBOW_STEP_PIN, LOW);
        delayMicroseconds(500);
    }
    elbowCurrentSteps = elbowTargetSteps;
}

