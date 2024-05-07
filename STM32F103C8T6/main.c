#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include <stdint.h>
#include <stdlib.h>

#include "capture.h"
#include "encoder.h"
#include "gpio.h"
#include "interrupt.h"
#include "oled.h"
#include "tim.h"

int16_t speed;

int main() {
    OLED_Init();

    TIM tim2 = {
        RCC_APB1Periph_TIM2, TIM2, TIM_InternalClock, 7200 - 1, 10000 - 1, CMD,
    };
    TIM_Init(&tim2, NULL);

    TIM_Interrupt interrupt = {
        TIM2, TIM2_IRQn, NVIC_PriorityGroup_2, 1, 1,
    };
    TIM_Interrupt_Init(&interrupt);

    GPIO gpio = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_6 | GPIO_Pin_7,
        GPIO_Mode_IPU,
    };
    TIM tim = {
        RCC_APB1Periph_TIM3, TIM3, NULL, 1 - 1, 65536 - 1, UNCMD,
    };
    Capture capture1 = {
        TIM3, TIM_Channel_1,   TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI,
        0xF,  TIM_GetCapture1,
    };
    Capture capture2 = {
        TIM3, TIM_Channel_2,   TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI,
        0xF,  TIM_GetCapture2,
    };
    Encoder encoder = {
        &gpio,
        &tim,
        &capture1,
        &capture2,
        TIM_ICPolarity_Rising,
        TIM_ICPolarity_Rising,
    };
    Encoder_Init(&encoder);

    for (;;) {
        OLED_ShowSignedNum(1, 1, speed, 5);
    }
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {

        speed = (int16_t)TIM_GetCounter(TIM3);
        TIM_SetCounter(TIM3, 0);

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}