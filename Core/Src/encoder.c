#include "encoder.h"
#include <math.h>

#define ENCODER_PULSES_PER_REV 2750

Encoder left_encoder;
Encoder right_encoder;

void Encoder_Init(Encoder *encoder, TIM_HandleTypeDef *htim) {
    encoder->htim = htim;
    encoder->last_encoder_value = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    encoder->last_time = HAL_GetTick();
    encoder->rpm = 0;
}

void Encoder_Calculate_RPM(Encoder *encoder) {
    uint32_t current_time = HAL_GetTick();
    int16_t current_encoder_value = (int16_t)__HAL_TIM_GET_COUNTER(encoder->htim);
    int16_t delta_encoder = current_encoder_value - encoder->last_encoder_value;

    if (delta_encoder > (ENCODER_PULSES_PER_REV / 2)) {
        delta_encoder -= ENCODER_PULSES_PER_REV;
    } else if (delta_encoder < -(ENCODER_PULSES_PER_REV / 2)) {
        delta_encoder += ENCODER_PULSES_PER_REV;
    }

    uint32_t delta_time = current_time - encoder->last_time;
    if (delta_time > 0) {
        encoder->rpm = (delta_encoder * 60000.0) / (ENCODER_PULSES_PER_REV * delta_time);
    }

    encoder->last_encoder_value = current_encoder_value;
    encoder->last_time = current_time;
}

void Encoder_Update(void) {
    Encoder_Calculate_RPM(&left_encoder);
    Encoder_Calculate_RPM(&right_encoder);
}
