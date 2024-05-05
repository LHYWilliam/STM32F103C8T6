#ifndef __CONTRAST_SENSOR_H
#define __CONTRAST_SENSOR_H

#include "gpio.h"

typedef struct {
    GPIO *gpio;
    uint8_t Mode;
} ContrastSensor;

void ContrastSensor_Init(ContrastSensor *sensor);
uint16_t ContrastSensor_Get();

#endif