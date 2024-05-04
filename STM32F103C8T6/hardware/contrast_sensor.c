#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "contrast_sensor.h"

void ContrastSensor_Init(ContrastSensor *sensor) {
    RCC_APB2PeriphClockCmd(sensor->RCC_APB2Periph, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {
        sensor->GPIO_Pin,
        GPIO_Speed_50MHz,
        GPIO_Mode_IPU,
    };
    GPIO_Init(sensor->GPIOx, &GPIO_InitStruct);
}