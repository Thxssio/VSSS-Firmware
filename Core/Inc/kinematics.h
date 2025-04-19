#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "motor_control.h"

#define WHEEL_RADIUS 0.0295
#define GEAR_RATIO 150.0

void Kinematics_Init(void);
float LinearToRPM(float v);
float RPMToLinear(double RPM);
void Kinematics_SetSpeeds(float vL, float vR);

#endif /* KINEMATICS_H */
