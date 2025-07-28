#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

// Pulley ratios (gear ratio: motor revolutions per joint revolution)
#define Q1_PULLEY_RATIO 20   // Base (q1) 1:20
#define Q2_PULLEY_RATIO 10   // Shoulder (q2) 1:10
#define Q3_PULLEY_RATIO 10   // Elbow (q3) 1:10

void initSteppers();
void moveToAngles(float q1, float q2, float q3);

long q1ToSteps(float q1);
long q2ToSteps(float q2);
long q3ToSteps(float q3);

#endif
