#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

void LED_Init(uint32_t RCC_APB2Periph, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
              GPIOSpeed_TypeDef GPIO_Speed, GPIOMode_TypeDef GPIO_Mode);
void LED_On(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void LED_Off(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void LED_Turn(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif