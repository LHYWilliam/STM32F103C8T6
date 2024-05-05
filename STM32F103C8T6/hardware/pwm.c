#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "pwm.h"

void PWM_Init(PWM *pwm) {
    RCC_APB2PeriphClockCmd(pwm->RCC_APB2Periph, ENABLE);

    TIM_OCInitTypeDef TIM_OCInitStruct = {
        .TIM_OCMode = TIM_OCMode_PWM1,
        .TIM_OCPolarity = TIM_OCPolarity_High,
        .TIM_OutputState = TIM_OutputState_Enable,
        .TIM_Pulse = pwm->TIM_Pulse,
    };
    pwm->TIM_OCInit(pwm->TIMx, &TIM_OCInitStruct);

    GPIO_InitTypeDef GPIO_InitStruct = {
        pwm->GPIO_Pin,
        GPIO_Speed_50MHz,
        pwm->GPIO_Mode,
    };
    GPIO_Init(pwm->GPIOx, &GPIO_InitStruct);
}