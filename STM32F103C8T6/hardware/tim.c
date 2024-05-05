#include "stm32f10x.h"

#include "tim.h"

void TIM_Init(TIM *tim, ClockSource_Config *config) {
    RCC_APB1PeriphClockCmd(tim->RCC_APB1Periph, ENABLE);

    tim->TIM_ClockSource(tim->TIMx, config);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
        tim->TIM_Prescaler - 1,
        TIM_CounterMode_Up,
        tim->TIM_Period - 1,
        TIM_CKD_DIV1,
        0,
    };
    TIM_TimeBaseInit(tim->TIMx, &TIM_TimeBaseInitStruct);

    TIM_Cmd(tim->TIMx, ENABLE);

    TIM_ClearFlag(tim->TIMx, TIM_FLAG_Update);
}

void TIM_InternalClock(TIM_TypeDef *TIMx, ClockSource_Config *config) {
    TIM_InternalClockConfig(TIMx);
}
void TIM_ETRClockMode2(TIM_TypeDef *TIMx, ClockSource_Config *config) {
    TIM_ETRClockMode2Config(TIMx, config->TIM_ExtTRGPrescaler,
                            config->TIM_ExtTRGPolarity, config->ExtTRGFilter);
};