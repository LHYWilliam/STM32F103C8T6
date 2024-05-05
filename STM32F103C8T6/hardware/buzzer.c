#include "stm32f10x_gpio.h"

#include "buzzer.h"

void Buzzer_Init(Buzzer *buzzer) { GPIO_Init_(buzzer->gpio); }

void Buzzer_On(Buzzer *buzzer) {
    GPIO_WriteBit(buzzer->gpio->GPIOx, buzzer->gpio->GPIO_Pin,
                  (BitAction)buzzer->Mode);
}

void Buzzer_Off(Buzzer *buzzer) {
    GPIO_WriteBit(buzzer->gpio->GPIOx, buzzer->gpio->GPIO_Pin,
                  (BitAction)(!buzzer->Mode));
}

void Buzzer_Turn(Buzzer *buzzer) {
    uint8_t now =
        GPIO_ReadOutputDataBit(buzzer->gpio->GPIOx, buzzer->gpio->GPIO_Pin);
    GPIO_WriteBit(buzzer->gpio->GPIOx, buzzer->gpio->GPIO_Pin,
                  (BitAction)(!now));
}