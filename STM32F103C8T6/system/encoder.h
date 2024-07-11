#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

typedef struct {
    char gpio[12];

    TIM_TypeDef *TIMx;
    uint16_t TIM_IC1Polarity;
    uint16_t TIM_IC2Polarity;
} Encoder;

void Encoder_Init(Encoder *encoder);

int16_t Encoder_Get(Encoder *encoder);

#endif