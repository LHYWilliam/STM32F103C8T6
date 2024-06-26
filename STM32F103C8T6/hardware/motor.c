#include "pwm.h"
#include "stm32f10x_gpio.h"

#include "motor.h"

void Motor_Init(Motor *motor) {
    PWM_Init(motor->pwm);
    GPIO_Init_(motor->gpio_direction1);
    GPIO_Init_(motor->gpio_direction2);
}
void Motor_Set(Motor *motor, int16_t speed) {
    GPIO_WriteBit(motor->gpio_direction1->GPIOx,
                  motor->gpio_direction1->GPIO_Pin,
                  speed >= 0 ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(motor->gpio_direction2->GPIOx,
                  motor->gpio_direction2->GPIO_Pin,
                  speed >= 0 ? Bit_SET : Bit_RESET);
    PWM_SetPulse(motor->pwm, speed >= 0 ? speed : -speed);
}