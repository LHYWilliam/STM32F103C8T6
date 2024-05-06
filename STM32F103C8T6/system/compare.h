#ifndef __COMPARE_H
#define __COMPARE_H

#include "stm32f10x.h"
#include <stdint.h>

typedef struct {
    TIM_TypeDef *TIMx;
    uint16_t TIM_Pulse;
    void (*TIM_OCInit)(TIM_TypeDef *TIMx, TIM_OCInitTypeDef *TIM_OCInitStruct);
    void (*TIM_SetCompare)(TIM_TypeDef *TIMx, uint16_t Compare1);
} Compare;

void Compare_Init(Compare *compare);
void Compare_Set(Compare *compare, uint16_t);

#endif