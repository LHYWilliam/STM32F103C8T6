#ifndef __MOTOR_H
#define __MOTOR_H

#include "pwm.h"
#include <stdint.h>

typedef struct {
    PWM *pwm;
    char direction1[4];
    char direction2[4];
} Motor;

void Motor_Init(Motor *motor);

void Motor_Set(Motor *motor, int16_t speed);

#endif