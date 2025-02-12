#include "VSSS.h"
#include <string.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;
extern UART_HandleTypeDef huart1;
VSSS_Robot robot;

uint8_t RxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
char msg[] = "VSSS Ready\r\n";


void VSSS_Init(void) {
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    Encoder_Init(&left_encoder, &htim3);
    Encoder_Init(&right_encoder, &htim4);

    Motor_Init(&motorLeft, &htim2, TIM_CHANNEL_1, INA1_GPIO_Port, INA1_Pin, INA2_GPIO_Port, INA2_Pin);
    Motor_Init(&motorRight, &htim1, TIM_CHANNEL_1, INB1_GPIO_Port, INB1_Pin, INB2_GPIO_Port, INB2_Pin);
    Kinematics_Init();
    NRF24_Init();
    NRF24_RxMode(RxAddress, 76);

    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
}

void VSSS_Run(void) {

	if (isDataAvailable(2) == 1) {
		NRF24_Receive(robot.RxData);
        memcpy(&robot.vL, &robot.RxData[0], sizeof(float));
        memcpy(&robot.vR, &robot.RxData[4], sizeof(float));
	}

	char data[50];
    snprintf(data, sizeof(data), "vL: %.2f, vR: %.2f\r\n", robot.vL, robot.vR);
    HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), 1000);
    Encoder_Update();
    Kinematics_SetSpeeds(robot.vL, robot.vR);

}
