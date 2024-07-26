#include <string.h>

#include "gpio.h"
#include "pwm.h"
#include "servo.h"

void Servo_Init(Servo *servo) {
    GPIO PWM_gpio = {
        .Mode = GPIO_Mode_AF_PP,
    };
    strcpy(PWM_gpio.GPIOxPiny, servo->PWM);
    GPIO_Init_(&PWM_gpio);

    PWM pwm = {
        .TIMx = servo->TIMx,
        .Prescaler = 72 - 1,
        .Period = 20000 - 1,
        .TIM_Init = servo->TIM_Init,
    };
    strcpy(pwm.channel, servo->channel);
    PWM_Init(&pwm);

    servo->TIM_SetCompare[0] = pwm.TIM_SetCompare[0];
    servo->TIM_SetCompare[1] = pwm.TIM_SetCompare[1];
}

void Servo_set(Servo *servo, float angle1, float angle2) {
    servo->TIM_SetCompare[0](servo->TIMx, angle1 / 180 * 2000 + 500);
    servo->TIM_SetCompare[1](servo->TIMx, angle2 / 180 * 2000 + 500);
}