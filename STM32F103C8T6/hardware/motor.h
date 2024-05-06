#ifndef __MOTOR_H
#define __MOTOR_H

#include "compare.h"
#include "gpio.h"
#include "tim.h"
#include <stdint.h>

typedef struct {
    TIM *tim;
    Compare *compare;
    GPIO *gpio_pwm;
    GPIO *gpio_direction1;
    GPIO *gpio_direction2;
} Motor;

void Motor_Init(Motor *motor);
void Motor_SetSpeed(Motor *motor, int8_t speed);

#endif