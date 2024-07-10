#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

#define RCC_APBxPeriph_TIMx(x)                                                 \
    ((x) == TIM1   ? RCC_APB2Periph_TIM1                                       \
     : (x) == TIM2 ? RCC_APB1Periph_TIM2                                       \
     : (x) == TIM3 ? RCC_APB1Periph_TIM3                                       \
     : (x) == TIM4 ? RCC_APB1Periph_TIM4                                       \
                   : NULL)

typedef struct {
    char gpio[12];

    TIM_TypeDef *TIMx;
    uint16_t TIM_IC1Polarity;
    uint16_t TIM_IC2Polarity;
} Encoder;

void Encoder_Init(Encoder *encoder);

int16_t Encoder_Get(Encoder *encoder);

#endif