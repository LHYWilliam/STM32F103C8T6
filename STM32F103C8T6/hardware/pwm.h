#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

typedef struct {
    TIM_TypeDef *TIMx;
    uint16_t TIM_Pulse;
    void (*TIM_OCInit)(TIM_TypeDef *TIMx, TIM_OCInitTypeDef *TIM_OCInitStruct);
    void (*TIM_SetCompare)(TIM_TypeDef *TIMx, uint16_t Compare1);
} PWM;

void PWM_Init(PWM *pwm);

#endif