#include <string.h>

#include "compare.h"
#include "pwm.h"
#include "tim.h"

void PWM_Init(PWM *pwm) {
    if (pwm->TIM_Init) {
        TIM tim = {
            .RCC_APBxPeriph = RCC_APBxPeriph_TIMx(pwm->TIMx),
            .TIMx = pwm->TIMx,
            .TIM_ClockSource = TIM_InternalClock,
            .TIM_Prescaler = pwm->Prescaler,
            .TIM_Period = pwm->Period,
            .CMD_Mode = CMD,
        };
        TIM_Init(&tim, NULL);
    }

    uint8_t count = 0;
    char *temp = pwm->channel;
    do {
        Compare compare = {
            .TIMx = pwm->TIMx,
            .TIM_Pulse = 0,
            .TIM_OCInit = TIM_OCxInit(temp[0] - '0'),
            .TIM_SetCompare = TIM_SetComparex(temp[0] - '0'),
        };
        Compare_Init(&compare);
        pwm->TIM_SetCompare[count] = compare.TIM_SetCompare;
    } while ((temp = strchr(temp, '|'), temp) && (temp = temp + 2) &&
             (count = count + 1));
}

void PWM_SetPrescaler(PWM *pwm, uint16_t prescaler) {
    TIM_PrescalerConfig(pwm->TIMx, prescaler, TIM_PSCReloadMode_Immediate);
}

void PWM_SetPulse(PWM *pwm, uint8_t channel, uint16_t pulse) {
    pwm->TIM_SetCompare[channel - 1](pwm->TIMx, pulse);
}