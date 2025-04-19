#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32g4xx_hal.h"

#define PWM_MAX  1699
#define PWM_MIN  0

typedef struct {
    TIM_HandleTypeDef *pwm_timer;
    uint32_t pwm_channel;
    GPIO_TypeDef *gpio_port1;
    uint16_t gpio_pin1;
    GPIO_TypeDef *gpio_port2;
    uint16_t gpio_pin2;
} Motor_t;

extern Motor_t motorLeft;
extern Motor_t motorRight;

void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *pwm_timer, uint32_t pwm_channel,
                GPIO_TypeDef *gpio_port1, uint16_t gpio_pin1,
                GPIO_TypeDef *gpio_port2, uint16_t gpio_pin2);

void Motor_Control(uint32_t pwm_left, uint8_t dir_left, uint32_t pwm_right, uint8_t dir_right);

#endif /* MOTOR_CONTROL_H */
