#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

#include <stdint.h>

typedef struct {
    char pwm[4];
    char direction1[4];
    char direction2[4];

    TIM_TypeDef *TIMx;
    uint8_t channel;
    uint8_t TIM_Init_Mode;

    GPIO_TypeDef *direction1_GPIOx;
    uint16_t direction1_GPIO_Pin;
    GPIO_TypeDef *direction2_GPIOx;
    uint16_t direction2_GPIO_Pin;

    void (*TIM_SetCompare)(TIM_TypeDef *TIMx, uint16_t Compare1);
} Motor;

void Motor_Init(Motor *motor);

void Motor_Set(Motor *motor, int16_t speed);

#endif