#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#include "stm32f10x_exti.h"
#include <stdint.h>

typedef struct {
    uint8_t GPIO_PortSource;
    uint8_t GPIO_PinSource;
    uint32_t EXTI_Line;
    EXTITrigger_TypeDef EXTI_Trigger;
    uint8_t NVIC_IRQChannel;
    uint32_t NVIC_PriorityGroup;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;

} GPIO_Interrut;

typedef struct {
    uint32_t RCC_APB1Periph;
    TIM_TypeDef *TIMx;
    void (*TIM_Source)(TIM_TypeDef *);
    uint16_t duration;
    uint8_t NVIC_IRQChannel;
    uint32_t NVIC_PriorityGroup;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;
} TIM_Interrupt;

void GPIO_Interrut_Init(GPIO_Interrut *interrupt);
void TIM_Interrupt_Init(TIM_Interrupt *interrupt);

#endif