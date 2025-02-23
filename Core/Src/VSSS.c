#include "VSSS.h"
#include "aes.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;
VSSS_Robot robot;

uint8_t RxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
char msg[] = "VSSS Ready\r\n";

#define AES_KEY_SIZE 16
#define AES_BLOCK_SIZE 16
#define ROBOT_ID 0

static const uint8_t aes_key[AES_KEY_SIZE] = {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
};

static uint8_t iv[AES_BLOCK_SIZE] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
};

struct AES_ctx ctx;

void VSSS_Init(void) {
    Kinematics_Init();
    NRF24_Init();
    NRF24_RxMode(RxAddress, 125);
    AES_init_ctx_iv(&ctx, aes_key, iv);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);

}

void VSSS_Run(void) {
    if (isDataAvailable(2) == 1) {
        NRF24_Receive(robot.RxData);
        uint8_t real_size = robot.RxData[0];

        AES_ctx_set_iv(&ctx, iv);
        AES_CBC_decrypt_buffer(&ctx, &robot.RxData[1], AES_BLOCK_SIZE);

//        char debug_buffer[100];
//        snprintf(debug_buffer, sizeof(debug_buffer), "Raw Data: %02X %02X %02X %02X\r\n",
//                 robot.RxData[1], robot.RxData[2], robot.RxData[3], robot.RxData[4]);
//        HAL_UART_Transmit(&huart1, (uint8_t*)debug_buffer, strlen(debug_buffer), 1000);

        if (real_size >= sizeof(int) + 2 * sizeof(float)) {

            memcpy(&robot.id, &robot.RxData[1], sizeof(int));
            robot.id = __builtin_bswap32(robot.id);
            memcpy(&robot.vL, &robot.RxData[5], sizeof(float));
            memcpy(&robot.vR, &robot.RxData[9], sizeof(float));

            if (robot.id == 0) {
                char buffer[50];
                snprintf(buffer, sizeof(buffer), "ID: %d, VL: %.2f, VR: %.2f\r\n", robot.id, robot.vL, robot.vR);
                HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
                Kinematics_SetSpeeds(robot.vL, robot.vR);
            } else {
                HAL_UART_Transmit(&huart1, (uint8_t*)"Message ignored (wrong ID)\r\n", 29, 1000);
                Kinematics_SetSpeeds(0.0, 0.0);
            }
        } else {
            HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid data received!\r\n", 24, 1000);
            Kinematics_SetSpeeds(0.0, 0.0);
        }
    }
}


