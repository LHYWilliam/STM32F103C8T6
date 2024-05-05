#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
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

void TIM_Interrupt_Init(TIM_Interrupt *interrupt, TIMClock_Config *config) {
    RCC_APB1PeriphClockCmd(interrupt->RCC_APB1Periph, ENABLE);

    interrupt->TIMClock_Source(interrupt->TIMx, config);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
        interrupt->TIM_Prescaler - 1,
        TIM_CounterMode_Up,
        interrupt->TIM_Period - 1,
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

void TIM_InternalClock(TIM_TypeDef *TIMx, TIMClock_Config *config) {
    TIM_InternalClockConfig(TIMx);
}
void TIM_ETRClockMode2(TIM_TypeDef *TIMx, TIMClock_Config *config) {
    RCC_APB2PeriphClockCmd(config->RCC_APB2Periph, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {
        config->GPIO_Pin,
        GPIO_Speed_50MHz,
        config->GPIO_Mode,
    };
    GPIO_Init(config->GPIOx, &GPIO_InitStruct);

    TIM_ETRClockMode2Config(TIMx, config->TIM_ExtTRGPrescaler,
                            config->TIM_ExtTRGPolarity, config->ExtTRGFilter);
};