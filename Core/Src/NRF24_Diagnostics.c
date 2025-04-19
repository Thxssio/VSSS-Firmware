#include "NRF24_Diagnostics.h"
#include "nrf24l01p.h"
#include <stdio.h>
#include <string.h>


static UART_HandleTypeDef *debug_uart = NULL;
extern SPI_HandleTypeDef hspi1;


void NRF24_Diagnostics_Init(UART_HandleTypeDef *uart) {
    debug_uart = uart;
}

void NRF24_UpdateStats(NRF24_Stats *stats) {
    uint8_t obs = 0;
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t[]){R_REGISTER | OBSERVE_TX}, &obs, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &obs, 1, HAL_MAX_DELAY);

    stats->lost_packets = (obs >> 4) & 0x0F;
    stats->retransmissions = obs & 0x0F;
    stats->link_quality = NRF24_CalculateLinkQuality(stats->lost_packets, stats->retransmissions);
}

float NRF24_CalculateLinkQuality(uint8_t lost, uint8_t retries) {
    // Fórmula simplificada — pode ajustar para refletir melhor sua aplicação
    float loss_factor = (float)lost / 15.0f;
    float retry_factor = (float)retries / 15.0f;

    float quality = 1.0f - (0.6f * loss_factor + 0.4f * retry_factor);
    if (quality < 0.0f) quality = 0.0f;
    return quality;
}

void NRF24_PrintStats(const NRF24_Stats *stats) {
    if (debug_uart == NULL) return;

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Lost: %d, Retries: %d, Quality: %.2f\r\n",
             stats->lost_packets, stats->retransmissions, stats->link_quality);
    HAL_UART_Transmit(debug_uart, (uint8_t*)buffer, strlen(buffer), 100);
}
