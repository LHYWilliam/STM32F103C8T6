#include "stm32f10x_gpio.h"

#include <stdint.h>
#include <string.h>

#include "compare.h"
#include "gpio.h"
#include "motor.h"
#include "tim.h"

void Motor_Init(Motor *motor) {
    GPIO PWM = {
        .GPIO_Mode = GPIO_Mode_AF_PP,
    };
    strcpy(PWM.GPIOxPiny, motor->PWM);
    GPIO_Init_(&PWM);

    GPIO IN1 = {
        .GPIO_Mode = GPIO_Mode_Out_PP,
    };
    strcpy(IN1.GPIOxPiny, motor->IN1);
    GPIO_Init_(&IN1);

    GPIO IN2 = {
        .GPIO_Mode = GPIO_Mode_Out_PP,
    };
    strcpy(IN2.GPIOxPiny, motor->IN2);
    GPIO_Init_(&IN2);

    motor->IN1_GPIOx = IN1.GPIOx;
    motor->IN1_GPIO_Pin = IN1.GPIO_Pin;
    motor->IN2_GPIOx = IN2.GPIOx;
    motor->IN2_GPIO_Pin = IN2.GPIO_Pin;

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

    motor->set_Mode = motor->invert ? Bit_SET : Bit_RESET;
}
void Motor_Set(Motor *motor, int16_t speed) {
    GPIO_WriteBit(motor->IN1_GPIOx, motor->IN1_GPIO_Pin,
                  speed >= 0 ? motor->set_Mode : !motor->set_Mode);
    GPIO_WriteBit(motor->IN2_GPIOx, motor->IN2_GPIO_Pin,
                  speed >= 0 ? !motor->set_Mode : motor->set_Mode);
    motor->TIM_SetCompare(motor->TIMx, (uint16_t)(speed >= 0 ? speed : -speed));
}