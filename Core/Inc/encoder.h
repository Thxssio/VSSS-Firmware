#ifndef ENCODER_H
#define ENCODER_H

#include "stm32g4xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    int16_t last_encoder_value;
    uint32_t last_time;
    float rpm;
} Encoder;

extern Encoder left_encoder;
extern Encoder right_encoder;

void Encoder_Init(Encoder *encoder, TIM_HandleTypeDef *htim);
void Encoder_Calculate_RPM(Encoder *encoder);
void Encoder_Update(void);

#endif /* ENCODER_H */
