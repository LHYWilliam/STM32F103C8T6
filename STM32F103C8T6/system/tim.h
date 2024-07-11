#ifndef __INIT_H
#define __INIT_H

#include "stm32f10x.h"

#define UNCMD ((uint8_t)0)
#define CMD ((uint8_t)1)

#define RCC_APBxPeriph_TIMx(x)                                                 \
    ((x) == TIM1   ? RCC_APB2Periph_TIM1                                       \
     : (x) == TIM2 ? RCC_APB1Periph_TIM2                                       \
     : (x) == TIM3 ? RCC_APB1Periph_TIM3                                       \
     : (x) == TIM4 ? RCC_APB1Periph_TIM4                                       \
                   : NULL)

typedef struct {
    uint16_t TIM_ExtTRGPrescaler;
    uint16_t TIM_ExtTRGPolarity;
    uint16_t ExtTRGFilter;
} ClockSource_Config;

typedef struct {
    uint32_t RCC_APBxPeriph;
    TIM_TypeDef *TIMx;
    void (*TIM_ClockSource)(TIM_TypeDef *TIMx, ClockSource_Config *config);
    uint16_t TIM_Prescaler;
    uint16_t TIM_Period;
    uint8_t CMD_Mode;
} TIM;

void TIM_Init(TIM *tim, ClockSource_Config *config);

void TIM_InternalClock(TIM_TypeDef *TIMx, ClockSource_Config *config);
void TIM_ETRClockMode2(TIM_TypeDef *TIMx, ClockSource_Config *config);

#endif