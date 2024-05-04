#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32f10x.h"

#define LOW ((uint8_t)0)
#define HIGH ((uint8_t)1)

typedef struct {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint8_t Mode;
} Buzzer;

void Buzzer_Init(uint32_t RCC_APB2Periph, Buzzer *buzzer);
void Buzzer_On(Buzzer *buzzer);
void Buzzer_Off(Buzzer *buzzer);
void Buzzer_Turn(Buzzer *buzzer);

#endif