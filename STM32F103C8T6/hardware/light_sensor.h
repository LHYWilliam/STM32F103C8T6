#ifndef __LIGHT_SENSOR_H
#define __LIGHT_SENSOR_H

#include "stm32f10x.h"

#define LOW ((uint8_t)0)
#define HIGH ((uint8_t)1)

typedef struct {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint8_t Mode;
} LightSensor;

void LightSensor_Init(LightSensor *sensor);
uint8_t LightSensor_Get(LightSensor *sensor);

#endif