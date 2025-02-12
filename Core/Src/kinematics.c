#include "kinematics.h"
#include "motor_control.h"
#include "encoder.h"
#include "PID.h"
#include <math.h>
#include "IMU.h"

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

int16_t accelData[3], gyroData[3];

char debug_imu[150];


void Kinematics_Init(void) {
	  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	  IMU_Init();

      Encoder_Init(&left_encoder, &htim3);
      Encoder_Init(&right_encoder, &htim4);

      Motor_Init(&motorLeft, &htim2, TIM_CHANNEL_1, INA1_GPIO_Port, INA1_Pin, INA2_GPIO_Port, INA2_Pin);
      Motor_Init(&motorRight, &htim1, TIM_CHANNEL_1, INB1_GPIO_Port, INB1_Pin, INB2_GPIO_Port, INB2_Pin);

	  PID2(&pidLeft, &inputLeft, &outputLeft, &setpoint_left_rpm, 0.5539, 124.0, 0.001194, _PID_CD_DIRECT);
	  PID2(&pidRight, &inputRight, &outputRight, &setpoint_right_rpm, 0.3515, 84.89, 0.001194, _PID_CD_DIRECT);

	  PID_SetOutputLimits(&pidLeft, -PWM_MAX, PWM_MAX);
	  PID_SetOutputLimits(&pidRight, -PWM_MAX, PWM_MAX);

	  PID_SetSampleTime(&pidLeft, 10);
	  PID_SetSampleTime(&pidRight, 10);

	  PID_SetMode(&pidLeft, _PID_MODE_AUTOMATIC);
	  PID_SetMode(&pidRight, _PID_MODE_AUTOMATIC);
}

/**
 * @brief Converte velocidade linear para RPM.
 */
float LinearToRPM(float v) {
    return (v * 60.0) / (2 * M_PI * WHEEL_RADIUS);
}

/**
 * @brief Define as velocidades do robô com base em velocidades lineares (m/s).
 *        Chama `Set_Motor_Speeds()` do `motor_control.c` para aplicar nos motores.
 */
void Kinematics_SetSpeeds(float vL, float vR) {
	Encoder_Update();


	uint8_t who_am_i = IMU_ReadReg(IMU_WHO_AM_I);
	IMU_ReadAccelData(accelData);
	IMU_ReadGyroData(gyroData);

	if (who_am_i == 0x70) {
		snprintf(debug_imu, sizeof(debug_imu),
				"Ax | RAW: %d, Ay | RAW: %d, Az | RAW: %d\r\n"
				"Gx | RAW: %d, Gy | RAW: %d, Gz | RAW: %d\r\n",
				accelData[0], accelData[1], accelData[2],
				gyroData[0], gyroData[1], gyroData[2]);
	}
	else {
		snprintf(debug_imu, sizeof(debug_imu), "Erro no IMU: Endereço não encontrado = 0x%X\r\n", who_am_i);
	}
	HAL_UART_Transmit(&huart1, (uint8_t*)debug_imu, strlen(debug_imu), HAL_MAX_DELAY);

	float target_rpm_left = LinearToRPM(vL);
    float target_rpm_right = LinearToRPM(vR);

    setpoint_left_rpm  = target_rpm_left;
    setpoint_right_rpm = target_rpm_right;

    inputLeft  = left_encoder.rpm;
    inputRight = right_encoder.rpm;


    PID_Compute(&pidLeft);
    PID_Compute(&pidRight);

    if (outputLeft > PWM_MAX) {
        outputLeft = PWM_MAX;
    } else if (outputLeft < -PWM_MAX) {
        outputLeft = -PWM_MAX;
    }

    if (outputRight > PWM_MAX) {
        outputRight = PWM_MAX;
    } else if (outputRight < -PWM_MAX) {
        outputRight = -PWM_MAX;
    }


    if (fabs(outputLeft) < OUTPUT_TOLERANCE) outputLeft = 0;
    if (fabs(outputRight) < OUTPUT_TOLERANCE) outputRight = 0;


    float pwm_left  = fabs(outputLeft);
    float pwm_right = fabs(outputRight);

    pwm_left  = fmax(pwm_left, PWM_MIN);
    pwm_left  = fmin(pwm_left, PWM_MAX);
    pwm_right = fmax(pwm_right, PWM_MIN);
    pwm_right = fmin(pwm_right, PWM_MAX);


    uint8_t dir_left  = (outputLeft >= 0) ? 0 : 1;
    uint8_t dir_right = (outputRight >= 0) ? 0 : 1;


    Motor_Control(pwm_left, dir_left, pwm_right, dir_right);

}
