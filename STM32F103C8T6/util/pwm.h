#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

typedef struct {
    TIM_TypeDef *TIMx;
    char channel[16];

    uint16_t Prescaler;
    uint16_t Period;

    void (*TIM_SetCompare[4])(TIM_TypeDef *TIMx, uint16_t Compare);

    uint8_t TIM_Init;
} PWM;

void PWM_Init(PWM *pwm);

void PWM_SetPrescaler(PWM *pwm, uint16_t val);
void PWM_SetPulse(PWM *pwm, uint8_t channel, uint16_t pulse);

#endif