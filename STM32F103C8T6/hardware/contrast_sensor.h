#ifndef __CONTRAST_SENSOR_H
#define __CONTRAST_SENSOR_H

#include "stm32f10x.h"

#define DOWN (uint8_t(0))
#define UP (uint8_t(1))
#define DOWN_UP (uint8_t(1))

typedef struct {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint8_t Mode;
} ContrastSensor;

void ContrastSensor_Init(uint32_t RCC_APB2Periph,
                         ContrastSensor *contrast_sensor);
uint16_t ContrastSensor_Get();

#endif