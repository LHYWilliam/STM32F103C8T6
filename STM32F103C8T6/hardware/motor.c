#include <stdint.h>
#include <string.h>

#include "gpio.h"
#include "motor.h"
#include "pwm.h"

void Motor_Init(Motor *motor) {
    GPIO PWM_gpio = {
        .Mode = GPIO_Mode_AF_PP,
    };
    strcpy(PWM_gpio.GPIOxPiny, motor->PWM);
    GPIO_Init_(&PWM_gpio);

    GPIO IN1 = {
        .Mode = GPIO_Mode_Out_PP,
    };
    strcpy(IN1.GPIOxPiny, motor->IN1);
    GPIO_Init_(&IN1);

    GPIO IN2 = {
        .Mode = GPIO_Mode_Out_PP,
    };
    strcpy(IN2.GPIOxPiny, motor->IN2);
    GPIO_Init_(&IN2);

    motor->IN1_GPIOx = IN1.GPIOx;
    motor->IN1_GPIO_Pin = IN1.GPIO_Pin;
    motor->IN2_GPIOx = IN2.GPIOx;
    motor->IN2_GPIO_Pin = IN2.GPIO_Pin;

    PWM pwm = {
        .TIMx = motor->TIMx,
        .Prescaler = 100 - 1,
        .Period = 7200 - 1,
        .TIM_Init = motor->TIM_Init,
    };
    strcpy(pwm.channel, motor->channel);
    PWM_Init(&pwm);

    motor->TIM_SetCompare = pwm.TIM_SetCompare[0];
    motor->set_Mode = motor->invert ? Bit_SET : Bit_RESET;
}
void Motor_Set(Motor *motor, int16_t speed) {
    GPIO_WriteBit(motor->IN1_GPIOx, motor->IN1_GPIO_Pin,
                  speed >= 0 ? motor->set_Mode : !motor->set_Mode);
    GPIO_WriteBit(motor->IN2_GPIOx, motor->IN2_GPIO_Pin,
                  speed >= 0 ? !motor->set_Mode : motor->set_Mode);
    motor->TIM_SetCompare(motor->TIMx, (uint16_t)(speed >= 0 ? speed : -speed));
}