#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "gpio.h"
#include "key.h"
#include "oled.h"
#include "pwm.h"
#include "servo.h"
#include "tim.h"

uint16_t counter;

int main() {
    OLED_Init();

    GPIO gpio1 = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_1,
        GPIO_Mode_IPU,
    };
    GPIO_Init_(&gpio1);
    Key key = {
        GPIOB,
        GPIO_Pin_1,
        LOW,
    };
    Key_Init(&key);

    TIM tim = {
        RCC_APB1Periph_TIM2, TIM2, TIM_InternalClock, 72 - 1, 20000 - 1,
    };
    PWM pwm = {
        TIM2,
        0,
        TIM_OC1Init,
        TIM_SetCompare1,
    };
    GPIO gpio2 = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_0,
        GPIO_Mode_AF_PP,
    };
    Servo servo = {
        &tim,
        &pwm,
        &gpio2,
    };

    Servo_Init(&servo);

    float angel = 0;
    for (;;) {
        if (Key_Read(&key)) {
            Servo_Set(&servo, angel);
            angel += 30;
            if (angel > 180) {
                angel = 0;
            }
        }
    }
}
