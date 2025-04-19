#include "VSSS.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;
VSSS_Robot robot;

uint8_t RxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
char msg[] = "VSSS Ready\r\n";

#define ROBOT_ID 0
#define COMMUNICATION_TIMEOUT_MS 300

static uint32_t last_rx_time = 0;
static uint32_t last_blink_time = 0;
static uint8_t led_state = 0;

static float vL = 0.0f;
static float vR = 0.0f;



void VSSS_Init(void) {
    Kinematics_Init();
    NRF24_Init();
    NRF24_RxMode(RxAddress, 125);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
    last_rx_time = HAL_GetTick();
}

void VSSS_Run(void) {
    if (isDataAvailable(2)) {
        NRF24_Receive(robot.RxData);
        VSSS_ProcessReceivedData();
    }

    if ((HAL_GetTick() - last_rx_time) > COMMUNICATION_TIMEOUT_MS) {
        Kinematics_SetSpeeds(0.0f, 0.0f);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }
//	Kinematics_SetSpeeds(vL, vR);
}

void VSSS_ProcessReceivedData(void) {
    uint8_t real_size = robot.RxData[0];
    const size_t expected_size = sizeof(int) + 2 * sizeof(float);

    if (real_size >= expected_size) {
        int id_raw;
        memcpy(&id_raw, &robot.RxData[1], sizeof(int));
        robot.id = __builtin_bswap32(id_raw);

        memcpy(&robot.vL, &robot.RxData[5], sizeof(float));
        memcpy(&robot.vR, &robot.RxData[9], sizeof(float));

        if (robot.id == ROBOT_ID) {
            last_rx_time = HAL_GetTick();
            Kinematics_SetSpeeds(robot.vL, robot.vR);
            VSSS_BlinkLED();
            VSSS_DebugOutput();
        }
    }
}

void VSSS_BlinkLED(void) {
    if ((HAL_GetTick() - last_blink_time) >= 200) {
        led_state = !led_state;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, led_state);
        last_blink_time = HAL_GetTick();
    }
}

void VSSS_DebugOutput(void) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "ID: %d, VL: %.2f, VR: %.2f\r\n", robot.id, robot.vL, robot.vR);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}






