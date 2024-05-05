#include "gpio.h"
#include "stm32f10x_gpio.h"

#include "light_sensor.h"

void LightSensor_Init(LightSensor *sensor) { GPIO_Init_(sensor->gpio); }

uint8_t LightSensor_Get(LightSensor *sensor) {
    return GPIO_ReadInputDataBit(sensor->gpio->GPIOx, sensor->gpio->GPIO_Pin) ==
           sensor->Mode;
}