#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

typedef struct {
    TIM_TypeDef *TIMx;
    uint16_t TIM_Pulse;
    void (*TIM_OCInit)(TIM_TypeDef *TIMx, TIM_OCInitTypeDef *TIM_OCInitStruct);

    uint32_t RCC_APB2Periph;
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    GPIOMode_TypeDef GPIO_Mode;
} PWM;

void PWM_Init(PWM *pwm);

#endif