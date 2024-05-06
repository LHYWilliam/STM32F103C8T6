#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "compare.h"
#include "delay.h"
#include "gpio.h"
#include "motor.h"
#include "tim.h"

uint16_t counter;

int main() {
    TIM tim = {
        RCC_APB1Periph_TIM2, TIM2, TIM_InternalClock, 720 - 1, 100 - 1,
    };
    Compare compare = {
        TIM2,
        0,
        TIM_OC1Init,
        TIM_SetCompare1,
    };
    GPIO gpio_pwm = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_0,
        GPIO_Mode_AF_PP,
    };
    GPIO gpio_direction1 = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_1,
        GPIO_Mode_Out_PP,
    };
    GPIO gpio_direction2 = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_2,
        GPIO_Mode_Out_PP,
    };
    Motor motor = {
        &tim, &compare, &gpio_pwm, &gpio_direction1, &gpio_direction2,
    };
    Motor_Init(&motor);

    for (;;) {
        for (int i = 0; i <= 100; i++) {
            Motor_SetSpeed(&motor, i);
            Delay_ms(10);
        }
        for (int i = 0; i <= 100; i++) {
            Motor_SetSpeed(&motor, 100 - i);
            Delay_ms(10);
        }
    }
}
