#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "led.h"

void LED_Init(uint32_t RCC_APB2Periph, LED *led) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {
        led->GPIO_Pin,
        GPIO_Speed_50MHz,
        GPIO_Mode_Out_PP,
    };
    GPIO_Init(led->GPIOx, &GPIO_InitStruct);
}

void LED_On(LED *led) {
    GPIO_WriteBit(led->GPIOx, led->GPIO_Pin, (BitAction)(led->Mode));
}

void LED_Off(LED *led) {
    GPIO_WriteBit(led->GPIOx, led->GPIO_Pin, (BitAction)(!led->Mode));
}

void LED_Turn(LED *led) {
    uint8_t now = GPIO_ReadOutputDataBit(led->GPIOx, led->GPIO_Pin);
    GPIO_WriteBit(led->GPIOx, led->GPIO_Pin, (BitAction)(!now));
}