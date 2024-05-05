#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include <stdlib.h>

#include "delay.h"
#include "gpio.h"
#include "oled.h"
#include "pwm.h"
#include "tim.h"

uint16_t counter;

int main() {
    OLED_Init();

    TIM tim = {
        RCC_APB1Periph_TIM2, TIM2, TIM_InternalClock, 720, 100,
    };
    TIM_Init(&tim, NULL);

    PWM pwm = {
        TIM2,
        0,
        TIM_OC1Init,
    };
    PWM_Init(&pwm);

    GPIO gpio = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_0,
        GPIO_Mode_AF_PP,
    };
    GPIO_Init_(&gpio);

    for (;;) {
        for (int i = 0; i <= 100; i++) {
            TIM_SetCompare1(TIM2, i);
            Delay_ms(10);
            OLED_ShowNum(1, 1, TIM_GetCapture1(TIM2), 3);
        }
        for (int i = 0; i <= 100; i++) {
            TIM_SetCompare1(TIM2, 100 - i);
            Delay_ms(10);
            OLED_ShowNum(1, 1, TIM_GetCapture1(TIM2), 3);
        }
    }
}
