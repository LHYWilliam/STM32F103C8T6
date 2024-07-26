#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>

#include "compare.h"
#include "tim.h"

typedef struct {
    TIM *tim;
    Compare *compare;
    char gpio[4];
    uint8_t Init_Mode;
} PWM;

void PWM_Init(PWM *pwm);

void PWM_SetPrescaler(PWM *pwm, uint16_t val);
void PWM_SetPulse(PWM *pwm, uint16_t val);

#endif