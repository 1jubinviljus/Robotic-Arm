#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H



// Target steps for each joint
extern long baseTargetSteps;
extern long shoulderTargetSteps;
extern long elbowTargetSteps;

// Current steps for each joint
extern long baseCurrentSteps;
extern long shoulderCurrentSteps;
extern long elbowCurrentSteps;


void moveSteppers(long baseAngle, long shoulderAngle, long elbowAngle);

#endif
