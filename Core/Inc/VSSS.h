#ifndef VSSS_H
#define VSSS_H

#include "main.h"
#include "nrf24l01p.h"
#include "kinematics.h"
#include "encoder.h"


typedef struct {
    float vL;
    float vR;
    uint8_t RxData[32];
} VSSS_Robot;


void VSSS_Init(void);
void VSSS_Run(void);

#endif /* VSSS_H */
