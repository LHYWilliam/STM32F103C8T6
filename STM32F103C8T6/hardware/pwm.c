#include "stm32f10x_tim.h"

#include "pwm.h"

void PWM_Init(PWM *pwm) {
    TIM_OCInitTypeDef TIM_OCInitStruct = {
        .TIM_OCMode = TIM_OCMode_PWM1,
        .TIM_OCPolarity = TIM_OCPolarity_High,
        .TIM_OutputState = TIM_OutputState_Enable,
        .TIM_Pulse = pwm->TIM_Pulse,
    };
    pwm->TIM_OCInit(pwm->TIMx, &TIM_OCInitStruct);
}