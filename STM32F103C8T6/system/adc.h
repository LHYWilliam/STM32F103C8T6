#ifndef __ADC_H
#define __ADC_H

#include "gpio.h"
#include <stdint.h>

typedef uint8_t ADC_Channels[];

typedef struct {
    GPIO *gpio;

    uint32_t RCC_APB2Periph;
    ADC_TypeDef *ADCx;
    uint8_t ADC_NbrOfChannel;
    uint8_t *ADC_Channel;
    FunctionalState ADC_ContinuousConvMode;
    uint32_t ADC_ExternalTrigConv;

    uint8_t DMA_Mode;
} ADC;

void ADC_Init_(ADC *adc);
uint16_t ADC_Get(ADC *adc);

#endif