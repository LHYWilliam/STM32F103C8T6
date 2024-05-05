#ifndef __INIT_H
#define __INIT_H

#include "stm32f10x.h"

typedef struct {
    uint32_t RCC_APB2Periph;
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    GPIOMode_TypeDef GPIO_Mode;
    uint16_t TIM_ExtTRGPrescaler;
    uint16_t TIM_ExtTRGPolarity;
    uint16_t ExtTRGFilter;
} Clock_Config;

typedef struct {
    uint32_t RCC_APB1Periph;
    TIM_TypeDef *TIMx;
    void (*TIM_ClockSource)(TIM_TypeDef *TIMx, Clock_Config *config);
    uint16_t TIM_Prescaler;
    uint16_t TIM_Period;
} TIM;

void TIM_Init(TIM *tim, Clock_Config *config);
void TIM_InternalClock(TIM_TypeDef *TIMx, Clock_Config *config);
void TIM_ETRClockMode2(TIM_TypeDef *TIMx, Clock_Config *config);

#endif