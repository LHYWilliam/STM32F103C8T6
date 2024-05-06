#include "compare.h"
#include "gpio.h"

#include "stdlib.h"

#include "pwm.h"

void PWM_Init(PWM *pwm) {
    TIM_Init(pwm->tim, NULL);
    Compare_Init(pwm->compare);
    GPIO_Init_(pwm->gpio);
}

void PWM_Set(PWM *pwm, uint16_t val) {
    pwm->compare->TIM_SetCompare(pwm->tim->TIMx, val);
}