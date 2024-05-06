#ifndef __SERVO_H
#define __SERVO_H

#include "pwm.h"

typedef struct {
    PWM *pwm;
} Servo;

void Servo_Init(Servo *servo);
void Servo_Set(Servo *servo, float angel);

#endif