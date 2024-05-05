#include <stdlib.h>

#include "gpio.h"
#include "pwm.h"
#include "servo.h"
#include "tim.h"

void Servo_Init(Servo *servo) {
    TIM_Init(servo->tim, NULL);
    PWM_Init(servo->pwm);
    GPIO_Init_(servo->gpio);
}

void Servo_Set(Servo *servo, float angel) {
    servo->pwm->TIM_SetCompare(servo->tim->TIMx, angel / 180 * 2000 + 500);
}