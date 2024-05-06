#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include <stdlib.h>

#include "capture.h"
#include "compare.h"
#include "gpio.h"
#include "oled.h"
#include "pwm.h"
#include "tim.h"

uint16_t counter;

int main() {
    OLED_Init();

    TIM tim2 = {
        RCC_APB1Periph_TIM2, TIM2, TIM_InternalClock, 720 - 1, 100 - 1,
    };
    Compare compare1 = {
        TIM2,
        50,
        TIM_OC1Init,
        TIM_SetCompare1,
    };
    GPIO gpio_pwm = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_0,
        GPIO_Mode_AF_PP,
    };
    PWM pwm = {
        &tim2,
        &compare1,
        &gpio_pwm,
    };
    PWM_Init(&pwm);

    GPIO gpio_capture = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_6,
        GPIO_Mode_IPU,
    };
    TIM tim3 = {
        RCC_APB1Periph_TIM3, TIM3, TIM_InternalClock, 72 - 1, 65536 - 1,
    };
    Capture freq = {
        TIM3, TIM_Channel_1,   TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI,
        0xF,  TIM_GetCapture1,
    };
    Capture duty = {
        TIM3,
        TIM_Channel_2,
        TIM_ICPolarity_Falling,
        TIM_ICSelection_IndirectTI,
        0xF,
        TIM_GetCapture2,
    };

    GPIO_Init_(&gpio_capture);
    TIM_Init(&tim3, NULL);
    Capture_Init(&freq);
    Capture_Init(&duty);

    TIM_SelectInputTrigger(freq.TIMx, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(freq.TIMx, TIM_SlaveMode_Reset);

    for (;;) {
        OLED_ShowNum(1, 1, Capture_GetFreq(&freq), 6);
        OLED_ShowNum(2, 1, Capture_GetDuty(&freq, &duty), 3);
    }
}
