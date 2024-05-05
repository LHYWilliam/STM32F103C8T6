#include "stm32f10x_gpio.h"
#include <stdint.h>

#include "light_sensor.h"

void LightSensor_Init(LightSensor *sensor) {}

uint8_t LightSensor_Get(LightSensor *sensor) {
    return GPIO_ReadInputDataBit(sensor->GPIOx, sensor->GPIO_Pin) ==
           sensor->Mode;
}