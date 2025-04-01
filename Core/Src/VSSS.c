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





// amostras

//#include "VSSS.h"
//#include <string.h>
//#include <stdio.h>
//#include <stdlib.h>  // Para gerar números aleatórios
//
//// Test
//#include "encoder.h"
//
//extern UART_HandleTypeDef huart1;
//VSSS_Robot robot;
//
//uint8_t RxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
//char msg[] = "VSSS Ready\r\n";
//
//#define ROBOT_ID 0
//
//// Variáveis para controle do tempo de execução e PWM
//uint32_t lastPWMTime = 0;  // Marca o último tempo em que o PWM foi alterado
//uint32_t currentTime = 0;  // Tempo atual (em ms)
//#define PWM_CHANGE_INTERVAL 10000  // Intervalo de 10 segundos para alterar o PWM (10.000 ms)
//uint32_t pwm_left = 0;  // Valor do PWM para o motor esquerdo
//
//// Função para gerar PWM aleatório
//uint32_t generate_random_pwm() {
//    return rand() % 1700;  // Gera um valor de PWM entre 0 e 1699
//}
//
//void VSSS_Init(void) {
//    Kinematics_Init();
//    NRF24_Init();
//    NRF24_RxMode(RxAddress, 125);
//    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
//    lastPWMTime = HAL_GetTick();  // Marca o tempo de início para alteração do PWM
//    pwm_left = generate_random_pwm(); // Inicializa o primeiro valor de PWM
//}
//
//void VSSS_Run(void) {
//    currentTime = HAL_GetTick();  // Atualiza o tempo atual
//
//    // Atualizar os encoders
//    Encoder_Update();
//    float vL_rpm = left_encoder.rpm;  // Coleta o RPM da roda esquerda
//
//    // Verifica se passaram 10 segundos para alterar o PWM
//    if (currentTime - lastPWMTime >= PWM_CHANGE_INTERVAL) {
//        pwm_left = generate_random_pwm();  // Gera um novo PWM aleatório
//        lastPWMTime = currentTime;  // Atualiza o tempo do último ajuste de PWM
//    }
//
//    // Controla os motores com o PWM constante durante os 10 segundos
//    Motor_Control(pwm_left, 0, 0, 0);  // Aplica o PWM gerado ao motor esquerdo e direito
//
//    // Formatar e enviar os dados via UART
//    char timeMsg[100];
//    snprintf(timeMsg, sizeof(timeMsg), "Time: %lu ms, PWM Left: %lu, RPM Left: %.2f\r\n",
//             (unsigned long)currentTime, (unsigned long)pwm_left, vL_rpm);
//    HAL_UART_Transmit(&huart1, (uint8_t*)timeMsg, strlen(timeMsg), 1000);
//}

