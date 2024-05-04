#ifndef __CONTRAST_SENSOR_H
#define __CONTRAST_SENSOR_H

#include "stm32f10x.h"

typedef struct {
    uint32_t RCC_APB2Periph;
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint8_t Mode;
} ContrastSensor;

void ContrastSensor_Init(ContrastSensor *sensor);
uint16_t ContrastSensor_Get();

#endif