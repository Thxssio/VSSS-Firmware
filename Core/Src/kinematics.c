#include "kinematics.h"
#include "motor_control.h"
#include "encoder.h"
#include "PID.h"
#include <math.h>

#include <string.h>
#include <stdio.h>

double setpoint_left_rpm  = 0.0;
double setpoint_right_rpm = 0.0;

double outputLeft  = 0.0;
double outputRight = 0.0;

double inputLeft   = 0.0;
double inputRight  = 0.0;

extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;
extern UART_HandleTypeDef huart1;

PID_TypeDef pidLeft, pidRight;

float accel[3], gyro[3];
char debug_imu[150];


void Kinematics_Init(void) {
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    Encoder_Init(&left_encoder, &htim3);
    Encoder_Init(&right_encoder, &htim4);

    Motor_Init(&motorLeft, &htim2, TIM_CHANNEL_1, INA1_GPIO_Port, INA1_Pin, INA2_GPIO_Port, INA2_Pin);
    Motor_Init(&motorRight, &htim1, TIM_CHANNEL_1, INB1_GPIO_Port, INB1_Pin, INB2_GPIO_Port, INB2_Pin);

    PID2(&pidLeft, &inputLeft, &outputLeft, &setpoint_left_rpm, 1.378, 390.3, 0.001194, _PID_CD_DIRECT);
    PID2(&pidRight, &inputRight, &outputRight, &setpoint_right_rpm, 1.378, 390.3, 0.001194, _PID_CD_DIRECT);

    PID_SetOutputLimits(&pidLeft, -PWM_MAX, PWM_MAX);
    PID_SetOutputLimits(&pidRight, -PWM_MAX, PWM_MAX);

    PID_SetSampleTime(&pidLeft, 1);
    PID_SetSampleTime(&pidRight, 1);

    PID_SetMode(&pidLeft, _PID_MODE_AUTOMATIC);
    PID_SetMode(&pidRight, _PID_MODE_AUTOMATIC);
}

/**
 * @brief Converte velocidade linear para RPM.
 */
float LinearToRPM(float v) {
	double rpm_motor = (v * 60.0f) / (2 * M_PI * WHEEL_RADIUS);
	return rpm_motor;
}

float RPMToLinear(double RPM){
	float linear_velocity = (RPM * 2 * M_PI * WHEEL_RADIUS) / 60.0f;
}

/**
 * @brief Define as velocidades do robô com base em velocidades lineares (m/s).
 */
void Kinematics_SetSpeeds(float vL, float vR) {
    Encoder_Update();

    float vL_real = RPMToLinear(left_encoder.rpm);
    float vR_real = RPMToLinear(right_encoder.rpm);

    float target_rpm_left = LinearToRPM(vL);
    float target_rpm_right = LinearToRPM(vR);

    setpoint_left_rpm  = target_rpm_left;
    setpoint_right_rpm = target_rpm_right;

    inputLeft  = left_encoder.rpm;
    inputRight = right_encoder.rpm;

    PID_Compute(&pidLeft);
    PID_Compute(&pidRight);

    Motor_Control(fabs(outputLeft), outputLeft >= 0 ? 0 : 1,
                  fabs(outputRight), outputRight >= 0 ? 0 : 1);


}
