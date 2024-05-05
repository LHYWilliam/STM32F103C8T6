#include "stm32f10x_gpio.h"

#include "led.h"

void LED_Init(LED *led) { GPIO_Init_(led->gpio); }

void LED_On(LED *led) {
    GPIO_WriteBit(led->gpio->GPIOx, led->gpio->GPIO_Pin,
                  (BitAction)(led->Mode));
}

void LED_Off(LED *led) {
    GPIO_WriteBit(led->gpio->GPIOx, led->gpio->GPIO_Pin,
                  (BitAction)(!led->Mode));
}

void LED_Turn(LED *led) {
    uint8_t now = GPIO_ReadOutputDataBit(led->gpio->GPIOx, led->gpio->GPIO_Pin);
    GPIO_WriteBit(led->gpio->GPIOx, led->gpio->GPIO_Pin, (BitAction)(!now));
}