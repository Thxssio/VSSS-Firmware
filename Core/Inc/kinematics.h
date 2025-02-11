#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "motor_control.h"

#define WHEEL_RADIUS 0.06

void Kinematics_Init(void);
float LinearToRPM(float v);
void Kinematics_SetSpeeds(float vL, float vR);

#endif /* KINEMATICS_H */
