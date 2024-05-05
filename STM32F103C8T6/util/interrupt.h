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

} GPIO_Interrut;

typedef struct {
    uint32_t RCC_APB2Periph;
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    GPIOMode_TypeDef GPIO_Mode;
    uint16_t TIM_ExtTRGPrescaler;
    uint16_t TIM_ExtTRGPolarity;
    uint16_t ExtTRGFilter;
} TIMClock_Config;

typedef struct {
    uint32_t RCC_APB1Periph;
    TIM_TypeDef *TIMx;
    void (*TIMClock_Source)(TIM_TypeDef *TIMx, TIMClock_Config *config);
    uint16_t TIM_Prescaler;
    uint16_t TIM_Period;
    uint8_t NVIC_IRQChannel;
    uint32_t NVIC_PriorityGroup;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;
} TIM_Interrupt;

void GPIO_Interrut_Init(GPIO_Interrut *interrupt);

void TIM_Interrupt_Init(TIM_Interrupt *interrupt, TIMClock_Config *config);
void TIM_InternalClock(TIM_TypeDef *TIMx, TIMClock_Config *config);
void TIM_ETRClockMode2(TIM_TypeDef *TIMx, TIMClock_Config *config);

#endif