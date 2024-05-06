#ifndef __PWM_H
#define __PWM_H

#include "compare.h"
#include "gpio.h"
#include "tim.h"
#include <stdint.h>

typedef struct {
    TIM *tim;
    Compare *compare;
    GPIO *gpio;
} PWM;

void PWM_Init(PWM *pwm);
void PWM_Set(PWM *pwm, uint16_t val);

#endif