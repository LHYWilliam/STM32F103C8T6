#ifndef __SERVO_H
#define __SERVO_H

#include "compare.h"
#include "gpio.h"
#include "tim.h"

typedef struct {
    TIM *tim;
    Compare *compare;
    GPIO *gpio;
} Servo;

void Servo_Init(Servo *servo);
void Servo_Set(Servo *servo, float angel);

#endif