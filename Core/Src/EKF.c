#include "EKF.h"
#include <math.h>
#include <string.h>

const float Q[4][4] = {
    {0.0001, 0, 0, 0},
    {0, 0.0001, 0, 0},
    {0, 0, 0.00005, 0},
    {0, 0, 0, 0.0002}
};


const float R[1][1] = {{0.01}};


float NormalizeAngle(float angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}


void EKF_Init(EKF_State *ekf) {
    ekf->x = 0.0;
    ekf->y = 0.0;
    ekf->theta = 0.0;
    ekf->v = 0.0;

    memset(ekf->P, 0, sizeof(ekf->P));
    ekf->P[0][0] = 0.1;
    ekf->P[1][1] = 0.1;
    ekf->P[2][2] = 0.1;
    ekf->P[3][3] = 0.1;
}


void EKF_Predict(EKF_State *ekf, float vL, float vR, float dt) {
    float v = (vR + vL) / 2.0;
    float omega = (vR - vL) / WHEEL_BASE;


    ekf->x += v * cos(ekf->theta) * dt;
    ekf->y += v * sin(ekf->theta) * dt;
    ekf->theta += omega * dt;
    ekf->v = v;


    ekf->theta = NormalizeAngle(ekf->theta);


    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ekf->P[i][j] += Q[i][j] * dt;
        }
    }
}


void EKF_Update(EKF_State *ekf, float theta_imu, float ax, float ay) {
    float K_theta;

    K_theta = 0.5;
    float theta_error = theta_imu - ekf->theta;
    ekf->theta += K_theta * theta_error;


    ekf->theta = NormalizeAngle(ekf->theta);
    ekf->P[2][2] += R[0][0];
}
