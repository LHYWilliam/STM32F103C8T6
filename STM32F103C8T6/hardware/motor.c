#include "stm32f10x_gpio.h"

#include <stdlib.h>

#include "motor.h"

void Motor_Init(Motor *motor) {
    TIM_Init(motor->tim, NULL);
    Compare_Init(motor->compare);
    GPIO_Init_(motor->gpio_pwm);
    GPIO_Init_(motor->gpio_direction1);
    GPIO_Init_(motor->gpio_direction2);
}
void Motor_SetSpeed(Motor *motor, int8_t speed) {
    GPIO_WriteBit(motor->gpio_direction1->GPIOx,
                  motor->gpio_direction1->GPIO_Pin,
                  speed >= 0 ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(motor->gpio_direction2->GPIOx,
                  motor->gpio_direction2->GPIO_Pin,
                  speed >= 0 ? Bit_RESET : Bit_SET);
    motor->compare->TIM_SetCompare(motor->tim->TIMx,
                                   speed >= 0 ? speed : -speed);
}