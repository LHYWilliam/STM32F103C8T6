#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "buzzer.h"

void Buzzer_Init(uint32_t RCC_APB2Periph, Buzzer *buzzer) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {
        buzzer->GPIO_Pin,
        GPIO_Speed_50MHz,
        GPIO_Mode_Out_PP,
    };
    GPIO_Init(buzzer->GPIOx, &GPIO_InitStruct);
}

void Buzzer_On(Buzzer *buzzer) {
    GPIO_WriteBit(buzzer->GPIOx, buzzer->GPIO_Pin, (BitAction)buzzer->Mode);
}

void Buzzer_Off(Buzzer *buzzer) {
    GPIO_WriteBit(buzzer->GPIOx, buzzer->GPIO_Pin, (BitAction)(!buzzer->Mode));
}

void Buzzer_Turn(Buzzer *buzzer) {
    uint8_t now = GPIO_ReadOutputDataBit(buzzer->GPIOx, buzzer->GPIO_Pin);
    GPIO_WriteBit(buzzer->GPIOx, buzzer->GPIO_Pin, (BitAction)(!now));
}