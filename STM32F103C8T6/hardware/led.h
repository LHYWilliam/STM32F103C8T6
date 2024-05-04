#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

#define LOW ((uint8_t)0)
#define HIGH ((uint8_t)01)

typedef struct {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint8_t Mode;
} LED;

void LED_Init(uint32_t RCC_APB2Periph, LED *led, GPIOSpeed_TypeDef GPIO_Speed,
              GPIOMode_TypeDef GPIO_Mode);
void LED_On(LED *led);
void LED_Off(LED *led);
void LED_Turn(LED *led);

#endif