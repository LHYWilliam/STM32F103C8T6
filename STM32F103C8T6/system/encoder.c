#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "capture.h"
#include "encoder.h"
#include "gpio.h"
#include "tim.h"

void Encoder_Init(Encoder *encoder) {
    GPIO gpio = {
        .Mode = GPIO_Mode_IPU,
    };
    strcpy(gpio.GPIOxPiny, encoder->gpio);
    GPIO_Init_(&gpio);

    TIM tim = {
        .TIMx = encoder->TIMx,
        .ClockSource = NULL,
        .Prescaler = 1 - 1,
        .Period = 65536 - 1,
        .CMD_Mode = UNCMD,
    };
    TIM_Init(&tim, NULL);

    Capture capture1 = {
        .TIMx = encoder->TIMx,
        .TIM_Channel = TIM_Channel_1,
        .TIM_ICPolarity = TIM_ICPolarity_Rising,
        .TIM_ICSelection = TIM_ICSelection_DirectTI,
        .TIM_ICFilter = 0xF,
        .TIM_GetCapture = TIM_GetCapture1,
    };
    Capture_Init(&capture1);

    Capture capture2 = {
        .TIMx = encoder->TIMx,
        .TIM_Channel = TIM_Channel_2,
        .TIM_ICPolarity = TIM_ICPolarity_Rising,
        .TIM_ICSelection = TIM_ICSelection_DirectTI,
        .TIM_ICFilter = 0xF,
        .TIM_GetCapture = TIM_GetCapture2,
    };
    Capture_Init(&capture2);

    TIM_EncoderInterfaceConfig(encoder->TIMx, TIM_EncoderMode_TI12,
                               encoder->invert ? TIM_ICPolarity_Falling
                                               : TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);

    TIM_Cmd(encoder->TIMx, ENABLE);
    TIM_ClearFlag(encoder->TIMx, TIM_FLAG_Update);
}

int16_t Encoder_Get(Encoder *encoder) {
    int16_t speed = (int16_t)TIM_GetCounter(encoder->TIMx);
    TIM_SetCounter(encoder->TIMx, 0);

    return speed;
}