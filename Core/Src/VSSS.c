#include "VSSS.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;
VSSS_Robot robot;

uint8_t RxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
char msg[] = "VSSS Ready\r\n";

//double left_pwm = 0.0;
//double left_rpm = 0.0;
//uint32_t timestamp = 0;
//uint32_t last_update_time = 0;


void VSSS_Init(void) {
    Kinematics_Init();
    NRF24_Init();
    NRF24_RxMode(RxAddress, 86);

    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);

//    srand(HAL_GetTick());
}

void VSSS_Run(void) {

	if (isDataAvailable(2) == 1) {
		NRF24_Receive(robot.RxData);
        memcpy(&robot.vL, &robot.RxData[0], sizeof(float));
        memcpy(&robot.vR, &robot.RxData[4], sizeof(float));
	}

//	char data[50];
//    snprintf(data, sizeof(data), "vL: %.2f, vR: %.2f\r\n", robot.vL, robot.vR);
//    HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), 1000);
    Kinematics_SetSpeeds(robot.vL, robot.vR);

}




//	timestamp = HAL_GetTick();
//	Encoder_Update();
//
//    if ((timestamp - last_update_time) >= 3000) {
//        left_pwm = (rand() % (1699 - 200 + 1)) + 200; // Gera um PWM aleatório entre 200 e 1699
//        last_update_time = timestamp; // Atualiza o tempo da última mudança de PWM
//    }
//
//    left_rpm = left_encoder.rpm;
//
//    char data[80];
//    snprintf(data, sizeof(data), "Time: %lu ms, left PWM: %.2f, left RPM: %.2f\r\n",
//             timestamp, left_pwm, left_rpm);
//    if(timestamp < 300000){
//       HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), 1000);
//       Motor_Control(left_pwm, 0, 0, 0);
//    }

