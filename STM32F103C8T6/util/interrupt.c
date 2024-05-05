#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "interrupt.h"

void GPIO_EXTIInterrut_Init(GPIO_EXTIInterrut *interrupt) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_EXTILineConfig(interrupt->GPIO_PortSource, interrupt->GPIO_PinSource);

    EXTI_InitTypeDef EXTI_InitStruct = {
        interrupt->EXTI_Line,
        EXTI_Mode_Interrupt,
        interrupt->EXTI_Trigger,
        ENABLE,
    };
    EXTI_Init(&EXTI_InitStruct);

    NVIC_PriorityGroupConfig(interrupt->NVIC_PriorityGroup);
    NVIC_InitTypeDef NVIC_InitStruct = {
        interrupt->NVIC_IRQChannel,
        interrupt->NVIC_IRQChannelPreemptionPriority,
        interrupt->NVIC_IRQChannelSubPriority,
        ENABLE,
    };
    NVIC_Init(&NVIC_InitStruct);
}

void Timer_Interrupt_Init() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
        7200 - 1, TIM_CounterMode_Up, 10000 - 1, TIM_CKD_DIV1, 0,
    };
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStruct = {
        TIM2_IRQn,
        2,
        1,
        ENABLE,
    };
    NVIC_Init(&NVIC_InitStruct);

    TIM_Cmd(TIM2, ENABLE);
}