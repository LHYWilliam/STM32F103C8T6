#include "stm32f10x_gpio.h"

#include "buzzer.h"

void Buzzer_Init(Buzzer *buzzer) {}

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