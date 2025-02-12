#include "motor_control.h"
#include <math.h>
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

Motor_t motorLeft;
Motor_t motorRight;

void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *pwm_timer, uint32_t pwm_channel,
                GPIO_TypeDef *gpio_port1, uint16_t gpio_pin1,
                GPIO_TypeDef *gpio_port2, uint16_t gpio_pin2) {
    motor->pwm_timer = pwm_timer;
    motor->pwm_channel = pwm_channel;
    motor->gpio_port1 = gpio_port1;
    motor->gpio_pin1 = gpio_pin1;
    motor->gpio_port2 = gpio_port2;
    motor->gpio_pin2 = gpio_pin2;

    HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_channel);
}

void Motor_Control(uint32_t pwm_left, uint8_t dir_left, uint32_t pwm_right, uint8_t dir_right) {
    /*
     - Motor Esquerdo: PWM no TIM2 CH1
       pinos de direção: PA1 (dir_left), PA2 (!dir_left) (exemplo)
     - Motor Direito: PWM no TIM1 CH1
       pinos de direção: PA9 (dir_right), PA10 (!dir_right) (exemplo)
     Ajuste conforme seu hardware.
    */

    // Motor Esquerdo
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_left);
    HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, (GPIO_PinState)(dir_left));
    HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, (GPIO_PinState)(!dir_left));

    // Motor Direito
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_right);
    HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin,  (GPIO_PinState)(dir_right));
    HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, (GPIO_PinState)(!dir_right));
}
