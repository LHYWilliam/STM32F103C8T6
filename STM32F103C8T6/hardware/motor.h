#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

#include <stdint.h>

typedef struct {
    char PWM[4];
    char IN1[4];
    char IN2[4];

    TIM_TypeDef *TIMx;
    uint8_t channel;
    uint8_t TIM_Init_Mode;

    uint8_t invert;
    BitAction set_Mode;

    GPIO_TypeDef *IN1_GPIOx;
    uint16_t IN1_GPIO_Pin;
    GPIO_TypeDef *IN2_GPIOx;
    uint16_t IN2_GPIO_Pin;

    void (*TIM_SetCompare)(TIM_TypeDef *TIMx, uint16_t Compare1);
} Motor;

void Motor_Init(Motor *motor);

void Motor_Set(Motor *motor, int16_t speed);

#endif