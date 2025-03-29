#include "VSSS.h"
//#include "aes.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;
VSSS_Robot robot;

uint8_t RxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
char msg[] = "VSSS Ready\r\n";

#define ROBOT_ID 0

void VSSS_Init(void) {
    Kinematics_Init();
    NRF24_Init();
    NRF24_RxMode(RxAddress, 125);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);

}


void VSSS_Run(void) {
    if (isDataAvailable(2) == 1) {
        NRF24_Receive(robot.RxData);
        uint8_t real_size = robot.RxData[0];

        if (real_size >= sizeof(int) + 2 * sizeof(float)) {

            memcpy(&robot.id, &robot.RxData[1], sizeof(int));
            robot.id = __builtin_bswap32(robot.id);
            memcpy(&robot.vL, &robot.RxData[5], sizeof(float));
            memcpy(&robot.vR, &robot.RxData[9], sizeof(float));

            if (robot.id == ROBOT_ID) {
                char buffer[50];
                snprintf(buffer, sizeof(buffer), "ID: %d, VL: %.2f, VR: %.2f\r\n", robot.id, robot.vL, robot.vR);
                HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
                Kinematics_SetSpeeds(robot.vL, robot.vR);
            } else {
                HAL_UART_Transmit(&huart1, (uint8_t*)"Message ignored (wrong ID)\r\n", 29, 1000);
            }
        } else {
            HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid data received!\r\n", 24, 1000);
        }
    }
}


