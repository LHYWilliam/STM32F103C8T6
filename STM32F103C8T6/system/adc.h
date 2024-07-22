#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

#define RCC_APB2Periph_ADCx(x)                                                 \
    ((x) == ADC1   ? RCC_APB2Periph_ADC1                                       \
     : (x) == ADC2 ? RCC_APB2Periph_ADC2                                       \
                   : NULL)

#define ADC_Channel_x(x)                                                       \
    ((x[0]) == '0'   ? ADC_Channel_0                                           \
     : (x[0]) == '1' ? ADC_Channel_1                                           \
     : (x[0]) == '2' ? ADC_Channel_2                                           \
     : (x[0]) == '3' ? ADC_Channel_3                                           \
     : (x[0]) == '4' ? ADC_Channel_4                                           \
     : (x[0]) == '5' ? ADC_Channel_5                                           \
     : (x[0]) == '6' ? ADC_Channel_6                                           \
     : (x[0]) == '7' ? ADC_Channel_7                                           \
     : (x[0]) == '8' ? ADC_Channel_8                                           \
     : (x[0]) == '9' ? ADC_Channel_9                                           \
                     : NULL)

typedef struct {
    char gpio[32];

    ADC_TypeDef *ADCx;
    char channel[32];

    uint8_t DMA;
} ADC;

void ADC_Init_(ADC *adc);
void ADC_Start(ADC *adc);

uint16_t ADC_Get(ADC *adc);

#endif