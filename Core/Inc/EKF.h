#ifndef EKF_H
#define EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <math.h>

#define WHEEL_BASE 0.06  // Distância entre as rodas (60 mm)


typedef struct {
    float x;
    float y;
    float theta;
    float v;
    float P[4][4];
} EKF_State;


void EKF_Init(EKF_State *ekf);
void EKF_Predict(EKF_State *ekf, float vL, float vR, float dt);
void EKF_Update(EKF_State *ekf, float theta_imu, float ax, float ay);
float NormalizeAngle(float angle);

#ifdef __cplusplus
}
#endif

#endif // EKF_H
