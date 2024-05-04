#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

#include "delay.h"
#include "key.h"

void Key_Init(uint32_t RCC_APB2Periph, Key *key) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {key->GPIO_Pin, GPIO_Speed_50MHz,
                                        GPIO_Mode_IPU};
    GPIO_Init(key->GPIOx, &GPIO_InitStruct);
}

uint8_t Key_Read(Key *key) {
    uint8_t if_key = 0;
    if (GPIO_ReadInputDataBit(key->GPIOx, key->GPIO_Pin) == key->Mode) {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(key->GPIOx, key->GPIO_Pin) == key->Mode)
            ;
        Delay_ms(20);
        if_key = 1;
    }
    return if_key;
}