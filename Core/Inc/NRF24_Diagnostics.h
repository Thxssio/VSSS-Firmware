#ifndef INC_NRF24_DIAGNOSTICS_H_
#define INC_NRF24_DIAGNOSTICS_H_

#include "main.h"

// Estrutura para armazenar estatísticas da comunicação
typedef struct {
    uint8_t lost_packets;
    uint8_t retransmissions;
    float link_quality;  // de 0.0 a 1.0
} NRF24_Stats;

void NRF24_Diagnostics_Init(UART_HandleTypeDef *uart);
void NRF24_UpdateStats(NRF24_Stats *stats);
void NRF24_PrintStats(const NRF24_Stats *stats);
float NRF24_CalculateLinkQuality(uint8_t lost, uint8_t retries);

#endif /* INC_NRF24_DIAGNOSTICS_H_ */
