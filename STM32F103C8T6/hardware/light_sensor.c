#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stdint.h>

#include "light_sensor.h"

void LightSensor_Init(LightSensor *sensor) {
    RCC_APB2PeriphClockCmd(sensor->RCC_APB2Periph, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {
        sensor->GPIO_Pin,
        GPIO_Speed_50MHz,
        GPIO_Mode_IPU,
    };
    GPIO_Init(sensor->GPIOx, &GPIO_InitStruct);
}

uint8_t LightSensor_Get(LightSensor *sensor) {
    return GPIO_ReadInputDataBit(sensor->GPIOx, sensor->GPIO_Pin) ==
           sensor->Mode;
}