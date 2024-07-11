#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"
#include <stdint.h>

typedef struct {
    char gpio[12];

    TIM_TypeDef *TIMx;

    uint8_t invert;
} Encoder;

void Encoder_Init(Encoder *encoder);

int16_t Encoder_Get(Encoder *encoder);

#endif