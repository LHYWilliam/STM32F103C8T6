#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f10x.h"

typedef struct {
    uint32_t RCC_APB2Periph;
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint8_t Mode;
} GPIO;

void GPIO_Init_(GPIO *gpio);

#endif