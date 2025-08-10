#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H


#define BASE_PULLEY_RATIO 20   // Base joint (Joint 1, X-axis driver) pulley ratio (1:20)
#define SHOULDER_PULLEY_RATIO 10 // Shoulder joint (Joint 2, Y-axis driver) pulley ratio (1:10)
#define ELBOW_PULLEY_RATIO 10    // Elbow joint (Joint 3, Z-axis driver) pulley ratio (1:10)

#define STEPS_PER_REV 200 // Typical stepper motor steps per revolution
#define MICROSTEPS 16 // Microstepping setting (e.g., 16 for 1/16 microstepping)

#define LOWER_ARM_LENGTH 0.2 // Length of the lower arm in meters
#define UPPER_ARM_LENGTH 0.15 // Length of the upper arm in meters
#define SHOULDER_GROUND_OFFSET 0.05 // Distance of the shoulder joint to the ground in meters

#define BASE_JOINT_MAX_ANGLE 120 // Maximum angle for the base joint in degrees
#define SHOULDER_JOINT_MAX_ANGLE 120 // Maximum angle for the shoulder joint
#define ELBOW_JOINT_MAX_ANGLE 180 // Maximum angle for the elbow joint in degrees


// Calculated steps per degree for each joint
extern long baseStepsPerDegree; // extern  tells the compiler that this variable is defined elsewhere
extern long shoulderStepsPerDegree;
extern long elbowStepsPerDegree;
#endif