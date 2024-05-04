#include "stm32f10x.h"

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