#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "key.h"
#include "led.h"
#include "oled.h"

int main() {
    LED_Init(RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_0, GPIO_Speed_50MHz,
             GPIO_Mode_Out_PP);
    Key_Init(RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_11, GPIO_Speed_50MHz,
             GPIO_Mode_IPU);

    for (;;) {
        if (Key_Read(GPIOB, GPIO_Pin_11, 0)) {
            LED_Turn(GPIOA, GPIO_Pin_0);
        }
    }
}
