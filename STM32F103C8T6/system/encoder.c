#include <stdlib.h>

#include "capture.h"
#include "encoder.h"
#include "gpio.h"
#include "stm32f10x_tim.h"
#include "tim.h"

void Encoder_Init(Encoder *encoder) {
    GPIO_Init_(encoder->gpio);
    TIM_Init(encoder->tim, NULL);
    Capture_Init(encoder->capture1);
    Capture_Init(encoder->capture2);

    TIM_EncoderInterfaceConfig(encoder->tim->TIMx, TIM_EncoderMode_TI12,
                               encoder->TIM_IC1Polarity,
                               encoder->TIM_IC2Polarity);

    TIM_Cmd(encoder->tim->TIMx, ENABLE);
    TIM_ClearFlag(encoder->tim->TIMx, TIM_FLAG_Update);
}