#include "compare.h"
#include "gpio.h"

#include "stdlib.h"
#include "stm32f10x_tim.h"

#include "pwm.h"

void PWM_Init(PWM *pwm) {
    TIM_Init(pwm->tim, NULL);
    Compare_Init(pwm->compare);
    GPIO_Init_(pwm->gpio);
}

void PWM_SetPrescaler(PWM *pwm, uint16_t prescaler) {
    TIM_PrescalerConfig(pwm->tim->TIMx, prescaler, TIM_PSCReloadMode_Immediate);
}

void PWM_SetPulse(PWM *pwm, uint16_t pulse) {
    pwm->compare->TIM_SetCompare(pwm->tim->TIMx, pulse);
}