#ifndef __SERVO_H
#define __SERVO_H

#include "gpio.h"
#include "pwm.h"
#include "tim.h"

typedef struct {
    TIM *tim;
    PWM *pwm;
    GPIO *gpio
} Servo;

void Servo_Init(Servo *servo);
void Servo_Set(Servo *servo, float angel);

#endif