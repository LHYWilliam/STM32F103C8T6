#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

typedef struct {
    char PWM[32];

    TIM_TypeDef *TIMx;
    char channel[32];

    void (*TIM_SetCompare[2])(TIM_TypeDef *TIMx, uint16_t Compare);
} Servo;

void Servo_Init(Servo *servo);

void Servo_set(Servo *servo, float angle1, float angle2);

#endif