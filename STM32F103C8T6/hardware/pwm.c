#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "pwm.h"

void PWM_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
        720 - 1, TIM_CounterMode_Up, 100 - 1, TIM_CKD_DIV1, 0,
    };
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

    TIM_OCInitTypeDef TIM_OCInitStruct = {
        .TIM_OCMode = TIM_OCMode_PWM1,
        .TIM_OCPolarity = TIM_OCPolarity_High,
        .TIM_OutputState = TIM_OutputState_Enable,
        .TIM_Pulse = 10,
    };
    TIM_OC1Init(TIM2, &TIM_OCInitStruct);

    TIM_Cmd(TIM2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {
        GPIO_Pin_0,
        GPIO_Speed_50MHz,
        GPIO_Mode_AF_PP,
    };
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}