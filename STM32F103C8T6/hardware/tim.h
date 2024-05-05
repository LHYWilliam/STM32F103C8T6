#ifndef __INIT_H
#define __INIT_H

#include "stm32f10x.h"

typedef struct {
    uint16_t TIM_ExtTRGPrescaler;
    uint16_t TIM_ExtTRGPolarity;
    uint16_t ExtTRGFilter;
} ClockSource_Config;

typedef struct {
    uint32_t RCC_APB1Periph;
    TIM_TypeDef *TIMx;
    void (*TIM_ClockSource)(TIM_TypeDef *TIMx, ClockSource_Config *config);
    uint16_t TIM_Prescaler;
    uint16_t TIM_Period;
} TIM;

void TIM_Init(TIM *tim, ClockSource_Config *config);
void TIM_InternalClock(TIM_TypeDef *TIMx, ClockSource_Config *config);
void TIM_ETRClockMode2(TIM_TypeDef *TIMx, ClockSource_Config *config);

#endif