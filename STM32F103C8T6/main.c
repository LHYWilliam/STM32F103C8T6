#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include <stdlib.h>

#include "delay.h"
#include "interrupt.h"
#include "oled.h"
#include "pwm.h"
#include "tim.h"

uint16_t counter;

int main() {
    OLED_Init();

    // TIM tim = {
    //     RCC_APB1Periph_TIM2, TIM2, TIM_InternalClock, 720, 100,
    // };
    // TIM_Init(&tim, NULL);

    // PWM pwm = {
    //     RCC_APB1Periph_TIM2,
    //     TIM2,
    //     720,
    //     100,
    //     0,
    //     TIM_OC1Init,

    //     RCC_APB2Periph_GPIOA,
    //     GPIOA,
    //     GPIO_Pin_0,
    //     GPIO_Mode_AF_PP,
    // };
    // PWM_Init(&pwm);

    // for (;;) {
    //     for (int i = 0; i <= 100; i++) {
    //         TIM_SetCompare1(TIM2, i);
    //         Delay_ms(10);
    //         OLED_ShowNum(1, 1, TIM_GetCapture1(TIM2), 3);
    //     }
    //     for (int i = 0; i <= 100; i++) {
    //         TIM_SetCompare1(TIM2, 100 - i);
    //         Delay_ms(10);
    //         OLED_ShowNum(1, 1, TIM_GetCapture1(TIM2), 3);
    //     }
    // }

    TIM tim = {
        RCC_APB1Periph_TIM2, TIM2, TIM_ETRClockMode2, 1, 10,
    };
    Clock_Config config = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_0,
        GPIO_Mode_IPU,
        TIM_ExtTRGPSC_OFF,
        TIM_ExtTRGPolarity_NonInverted,
        0x0f,
    };
    TIM_Init(&tim, &config);

    TIM_Interrupt interrupt = {
        TIM2, TIM2_IRQn, NVIC_PriorityGroup_2, 1, 1,
    };
    TIM_Interrupt_Init(&interrupt);

    for (;;) {
        OLED_ShowNum(1, 1, TIM_GetCounter(TIM2), 3);
    }
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        counter++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}