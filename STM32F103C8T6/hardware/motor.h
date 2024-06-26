#ifndef __MOTOR_H
#define __MOTOR_H

#include "gpio.h"
#include "pwm.h"
#include <stdint.h>

typedef struct {
    PWM *pwm;
    GPIO *gpio_direction1;
    GPIO *gpio_direction2;
} Motor;

void Motor_Init(Motor *motor);

void Motor_Set(Motor *motor, int16_t speed);

#endif