#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#include "stm32f10x_exti.h"

typedef struct {
    uint8_t GPIO_PortSource;
    uint8_t GPIO_PinSource;
    uint32_t EXTI_Line;
    EXTITrigger_TypeDef EXTI_Trigger;
    uint8_t NVIC_IRQChannel;
    uint32_t NVIC_PriorityGroup;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;

} GPIO_EXTIInterrut;

void GPIO_EXTIInterrut_Init(GPIO_EXTIInterrut *interrupt);

#endif