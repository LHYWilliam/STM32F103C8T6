#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "interrupt.h"

void GPIO_Interrut_Init(GPIO_Interrut *interrupt) {
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

void TIM_Interrupt_Init(TIM_Interrupt *interrupt) {
    RCC_APB1PeriphClockCmd(interrupt->RCC_APB1Periph, ENABLE);

    interrupt->TIM_Source(interrupt->TIMx);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
        7200 - 1,
        TIM_CounterMode_Up,
        interrupt->duration * 10 - 1,
        TIM_CKD_DIV1,
        0,
    };
    TIM_TimeBaseInit(interrupt->TIMx, &TIM_TimeBaseInitStruct);

    TIM_ClearFlag(interrupt->TIMx, TIM_FLAG_Update);

    TIM_ITConfig(interrupt->TIMx, TIM_IT_Update, ENABLE);

    NVIC_PriorityGroupConfig(interrupt->NVIC_PriorityGroup);

    NVIC_InitTypeDef NVIC_InitStruct = {
        interrupt->NVIC_IRQChannel,
        interrupt->NVIC_IRQChannelPreemptionPriority,
        interrupt->NVIC_IRQChannelSubPriority,
        ENABLE,
    };
    NVIC_Init(&NVIC_InitStruct);

    TIM_Cmd(interrupt->TIMx, ENABLE);
}