#include "stm32f10x_gpio.h"

#include "gpio.h"
#include "motor.h"
#include "pwm.h"
#include "tim.h"
#include <time.h>

void Motor_Init(Motor *motor) {
    TIM_Init(motor->tim, NULL);
    PWM_Init(motor->pwm);
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
    motor->pwm->TIM_SetCompare(motor->tim->TIMx, speed >= 0 ? speed : -speed);
}