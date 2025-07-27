#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

// set up the stepper motor pins
void initSteppers();

// move the stepper motor to a specific angle
long anglesToSteps(long q1, long q2, long q3);

// converts angle about the base to motor steps
long q1ToSteps(long q1);

// converts angle about the shoulder joint to motor steps
long q2ToSteps(long q2);

// converts angle about the elbow joint to motor steps
long q3ToSteps(long q3);

#endif