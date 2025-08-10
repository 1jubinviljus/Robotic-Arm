#include "robot_parameters.h"

// Calculate steps per degree for each joint
long baseStepsPerDegree = (STEPS_PER_REV * MICROSTEPS * BASE_PULLEY_RATIO) / 360;
long shoulderStepsPerDegree = (STEPS_PER_REV * MICROSTEPS * SHOULDER_PULLEY_RATIO) / 360;
long elbowStepsPerDegree = (STEPS_PER_REV * MICROSTEPS * ELBOW_PULLEY_RATIO) / 360;