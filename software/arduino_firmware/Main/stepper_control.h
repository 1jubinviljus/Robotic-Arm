#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

// Pulley ratios (gear ratio: motor revolutions per joint revolution)
#define Q1_PULLEY_RATIO 20   // Base (q1) 1:20
#define Q2_PULLEY_RATIO 10   // Shoulder (q2) 1:10
#define Q3_PULLEY_RATIO 10   // Elbow (q3) 1:10

// Motor steps per revolution
#define MICROSTEPS 16 // Microstepping setting (e.g., 16 for 1/16 microstepping)

void moveSteppers();

#endif
