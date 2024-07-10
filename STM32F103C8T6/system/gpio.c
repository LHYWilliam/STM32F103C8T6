#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include <stdlib.h>
#include <string.h>

#include "gpio.h"

void GPIO_Init_(GPIO *gpio) {
    char *temp = gpio->GPIOxPiny;
    do {
        gpio->GPIOx = GPIOx(temp);
        gpio->GPIO_Pin = GPIO_Pinx(temp);

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOx(temp), ENABLE);

        GPIO_InitTypeDef GPIO_InitStruct = {
            gpio->GPIO_Pin,
            GPIO_Speed_50MHz,
            gpio->GPIO_Mode,
        };
        GPIO_Init(gpio->GPIOx, &GPIO_InitStruct);
    } while ((temp = strchr(temp, '|'), temp) && (temp = temp + 2));
}