#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "gpio.h"

void GPIO_Init_(GPIO *gpio) {
    RCC_APB2PeriphClockCmd(gpio->RCC_APB2Periph, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {
        gpio->GPIO_Pin,
        GPIO_Speed_50MHz,
        gpio->Mode,
    };
    GPIO_Init(gpio->GPIOx, &GPIO_InitStruct);
}