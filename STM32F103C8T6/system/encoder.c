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
        .GPIO_Mode = GPIO_Mode_IPU,
    };
    strcpy(gpio.GPIOxPiny, encoder->gpio);
    GPIO_Init_(&gpio);

    TIM_Init(encoder->tim, NULL);
    Capture_Init(encoder->capture1);
    Capture_Init(encoder->capture2);

    TIM_EncoderInterfaceConfig(encoder->tim->TIMx, TIM_EncoderMode_TI12,
                               encoder->TIM_IC1Polarity,
                               encoder->TIM_IC2Polarity);

    TIM_Cmd(encoder->tim->TIMx, ENABLE);
    TIM_ClearFlag(encoder->tim->TIMx, TIM_FLAG_Update);
}

int16_t Encoder_Get(Encoder *encoder) {
    int16_t speed = (int16_t)TIM_GetCounter(encoder->tim->TIMx);
    TIM_SetCounter(encoder->tim->TIMx, 0);

    return speed;
}