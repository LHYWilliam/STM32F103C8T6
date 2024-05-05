#include "stm32f10x.h"
#include "stm32f10x_tim.h"

#include "delay.h"
#include "oled.h"
#include "pwm.h"

int main() {
    OLED_Init();
    PWM_Init();

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