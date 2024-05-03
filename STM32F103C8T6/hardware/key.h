#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

void Key_Init(uint32_t RCC_APB2Periph, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
              GPIOSpeed_TypeDef GPIO_Speed, GPIOMode_TypeDef GPIO_Mode);

uint8_t Key_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t Mode);

#endif