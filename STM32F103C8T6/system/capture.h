#ifndef __CAPTURE_H
#define __CAPTURE_H

#include "stm32f10x.h"

typedef struct {
    TIM_TypeDef *TIMx;
    uint16_t TIM_Channel;
    uint16_t TIM_ICPolarity;
    uint16_t TIM_ICSelection;
    uint16_t TIM_ICFilter;
    uint16_t (*TIM_GetCapture)(TIM_TypeDef *TIMx);
} Capture;

void Capture_Init(Capture *capture);

#endif