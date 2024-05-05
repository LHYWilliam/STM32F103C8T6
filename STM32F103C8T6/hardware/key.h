#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

#define LOW ((uint8_t)0)
#define HIGH ((uint8_t)1)

typedef struct {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint8_t Mode;
} Key;

void Key_Init(Key *key);
uint8_t Key_Read(Key *key);

#endif