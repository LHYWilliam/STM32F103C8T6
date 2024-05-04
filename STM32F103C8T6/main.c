#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "key.h"
#include "led.h"

int main() {
    LED led = {GPIOA, GPIO_Pin_0, HIGH};
    LED_Init(RCC_APB2Periph_GPIOA, &led, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);

    Key key = {GPIOB, GPIO_Pin_11, LOW};
    Key_Init(RCC_APB2Periph_GPIOB, &key, GPIO_Speed_50MHz, GPIO_Mode_IPU);

    for (;;) {
        if (Key_Read(&key)) {
            LED_Turn(&led);
        }
    }
}
