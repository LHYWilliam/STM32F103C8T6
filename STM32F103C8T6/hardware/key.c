#include "stm32f10x_gpio.h"

#include "delay.h"
#include "key.h"

void Key_Init(Key *key) {}

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