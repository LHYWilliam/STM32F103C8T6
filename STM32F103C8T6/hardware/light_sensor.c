#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stdint.h>

#include "light_sensor.h"

void LightSensor_Init(uint32_t RCC_APB2Periph, LightSensor *light_sensor) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {
        light_sensor->GPIO_Pin,
        GPIO_Speed_50MHz,
        GPIO_Mode_IPU,
    };
    GPIO_Init(light_sensor->GPIOx, &GPIO_InitStruct);
}

uint8_t LightSensor_Get(LightSensor *light_sensor) {
    return GPIO_ReadInputDataBit(light_sensor->GPIOx, light_sensor->GPIO_Pin) ==
           light_sensor->Mode;
}