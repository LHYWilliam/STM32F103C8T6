#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "interrupt.h"
#include "oled.h"

uint16_t counter;

void EXTI15_10_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line12) == SET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
            counter++;
        }
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}

int main() {
    OLED_Init();

    // GPIO_Interrut interrupt = {
    //     GPIO_PortSourceGPIOB,
    //     GPIO_PinSource12,
    //     EXTI_Line12,
    //     EXTI_Trigger_Falling,
    //     EXTI15_10_IRQn,
    //     NVIC_PriorityGroup_2,
    //     1,
    //     1,
    // };
    // GPIO_Interrut_Init(&interrupt);

    TIM_Interrupt interrupt = {
        RCC_APB1Periph_TIM2,
        TIM2,
        TIM_InternalClockConfig,
        1000,
        TIM2_IRQn,
        NVIC_PriorityGroup_2,
        1,
        1,
    };
    TIM_Interrupt_Init(&interrupt);

    for (;;) {
        OLED_ShowNum(1, 1, counter, 3);
    }
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        counter++;

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
