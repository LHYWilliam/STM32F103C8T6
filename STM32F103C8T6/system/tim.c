#include "stm32f10x.h"

#include <stdlib.h>

#include "interrupt.h"
#include "tim.h"

void TIM_Init(TIM *tim, ClockSource_Config *config) {
    if (tim->TIMx == TIM1) {
        RCC_APB2PeriphClockCmd(tim->RCC_APBxPeriph, ENABLE);
    } else {
        RCC_APB1PeriphClockCmd(tim->RCC_APBxPeriph, ENABLE);
    }

    if (tim->TIM_ClockSource) {
        tim->TIM_ClockSource(tim->TIMx, config);
    }

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
        tim->TIM_Prescaler,
        TIM_CounterMode_Up,
        tim->TIM_Period,
        TIM_CKD_DIV1,
        0,
    };
    TIM_TimeBaseInit(tim->TIMx, &TIM_TimeBaseInitStruct);

    if (tim->TIMx == TIM1) {
        TIM_CtrlPWMOutputs(tim->TIMx, ENABLE);
    }

    if (tim->CMD_Mode) {
        TIM_Cmd(tim->TIMx, ENABLE);
        TIM_ClearFlag(tim->TIMx, TIM_FLAG_Update);
    }
}

void Timer_Init(Timer *timer) {
    TIM tim = {
        .RCC_APBxPeriph = RCC_APBxPeriph_TIMx(timer->TIMx),
        .TIMx = timer->TIMx,
        .TIM_ClockSource = TIM_InternalClock,
        .TIM_Prescaler = 7200 - 1,
        .TIM_Period = timer->ms * 10 - 1,
        .CMD_Mode = UNCMD,
    };
    TIM_Init(&tim, NULL);

    TIM_Interrupt TIM_interrupt = {
        .TIMx = timer->TIMx,
        .NVIC_IRQChannel = TIMx_IRQn(timer->TIMx),
        .NVIC_PriorityGroup = NVIC_PriorityGroup_2,
        .NVIC_IRQChannelPreemptionPriority = 0,
        .NVIC_IRQChannelSubPriority = 2,
    };
    TIM_Interrupt_Init(&TIM_interrupt);
}

void TIM_InternalClock(TIM_TypeDef *TIMx, ClockSource_Config *config) {
    TIM_InternalClockConfig(TIMx);
}
void TIM_ETRClockMode2(TIM_TypeDef *TIMx, ClockSource_Config *config) {
    TIM_ETRClockMode2Config(TIMx, config->TIM_ExtTRGPrescaler,
                            config->TIM_ExtTRGPolarity, config->ExtTRGFilter);
};