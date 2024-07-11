#include "stm32f10x_gpio.h"

#include <stdint.h>
#include <string.h>

#include "compare.h"
#include "gpio.h"
#include "motor.h"
#include "tim.h"

void Motor_Init(Motor *motor) {
    GPIO pwm = {
        .GPIO_Mode = GPIO_Mode_AF_PP,
    };
    strcpy(pwm.GPIOxPiny, motor->pwm);
    GPIO_Init_(&pwm);

    GPIO direction1 = {
        .GPIO_Mode = GPIO_Mode_Out_PP,
    };
    strcpy(direction1.GPIOxPiny, motor->direction1);
    GPIO_Init_(&direction1);

    GPIO direction2 = {
        .GPIO_Mode = GPIO_Mode_Out_PP,
    };
    strcpy(direction2.GPIOxPiny, motor->direction2);
    GPIO_Init_(&direction2);

    motor->direction1_GPIOx = direction1.GPIOx;
    motor->direction1_GPIO_Pin = direction1.GPIO_Pin;
    motor->direction2_GPIOx = direction2.GPIOx;
    motor->direction2_GPIO_Pin = direction2.GPIO_Pin;

    if (motor->TIM_Init_Mode) {
        TIM tim = {
            .RCC_APBxPeriph = RCC_APBxPeriph_TIMx(motor->TIMx),
            .TIMx = motor->TIMx,
            .TIM_ClockSource = TIM_InternalClock,
            .TIM_Prescaler = 100 - 1,
            .TIM_Period = 7200 - 1,
            .CMD_Mode = CMD,
        };
        TIM_Init(&tim, NULL);
    }

    Compare compare = {
        .TIMx = motor->TIMx,
        .TIM_Pulse = 0,
        .TIM_OCInit = TIM_OCxInit(motor->channel),
        .TIM_SetCompare = TIM_SetComparex(motor->channel),
    };
    Compare_Init(&compare);

    motor->TIM_SetCompare = compare.TIM_SetCompare;
}
void Motor_Set(Motor *motor, int16_t speed) {
    GPIO_WriteBit(motor->direction1_GPIOx, motor->direction1_GPIO_Pin,
                  speed >= 0 ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(motor->direction2_GPIOx, motor->direction2_GPIO_Pin,
                  speed >= 0 ? Bit_SET : Bit_RESET);
    motor->TIM_SetCompare(motor->TIMx, (uint16_t)(speed >= 0 ? speed : -speed));
}