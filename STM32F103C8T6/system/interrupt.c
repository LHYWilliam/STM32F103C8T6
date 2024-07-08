#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

#include "interrupt.h"

void GPIO_Interrupt_Init(GPIO_Interrut *interrupt) {
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

void USART_Interrupt_Init(USART_Interrupt *interrupt) {
    USART_ITConfig(interrupt->USARTx, interrupt->USART_IT, ENABLE);

    NVIC_PriorityGroupConfig(interrupt->NVIC_PriorityGroup);

    NVIC_InitTypeDef NVIC_InitStruct = {
        interrupt->NVIC_IRQChannel,
        interrupt->NVIC_IRQChannelPreemptionPriority,
        interrupt->NVIC_IRQChannelSubPriority,
        ENABLE,
    };
    NVIC_Init(&NVIC_InitStruct);
}