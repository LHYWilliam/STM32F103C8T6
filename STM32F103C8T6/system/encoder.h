#ifndef __ENCODER_H
#define __ENCODER_H

#include "capture.h"
#include "gpio.h"
#include "tim.h"

typedef struct {
    GPIO *gpio;
    TIM *tim;
    Capture *capture1;
    Capture *capture2;
    uint16_t TIM_IC1Polarity;
    uint16_t TIM_IC2Polarity
} Encoder;

void Encoder_Init(Encoder *encoder);

#endif