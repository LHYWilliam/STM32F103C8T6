#include <stdlib.h>

#include "pwmi.h"

void PWMI_Init(PWMI *pwmi) {
    GPIO_Init_(pwmi->gpio);
    TIM_Init(pwmi->tim, NULL);
    Capture_Init(pwmi->frequency);
    Capture_Init(pwmi->duty);

    TIM_SelectInputTrigger(pwmi->tim->TIMx, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(pwmi->tim->TIMx, TIM_SlaveMode_Reset);

    TIM_Cmd(pwmi->tim->TIMx, ENABLE);
    TIM_ClearFlag(pwmi->tim->TIMx, TIM_FLAG_Update);
}
uint16_t PWMI_GetFrequency(PWMI *pwmi) {
    return 1000000 /
           (pwmi->frequency->TIM_GetCapture(pwmi->frequency->TIMx) + 1);
}
uint16_t PWMI_GetDuty(PWMI *pwmi) {
    return (pwmi->duty->TIM_GetCapture(pwmi->tim->TIMx) + 1) * 100 /
           (pwmi->frequency->TIM_GetCapture(pwmi->tim->TIMx) + 1);
}