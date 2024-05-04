#include "buzzer.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "key.h"
#include "led.h"

int main() {
    LED led = {GPIOA, GPIO_Pin_0, HIGH};
    LED_Init(RCC_APB2Periph_GPIOA, &led);

    Key led_key = {GPIOB, GPIO_Pin_11, LOW};
    Key_Init(RCC_APB2Periph_GPIOB, &led_key);

    Buzzer buzzer = {GPIOB, GPIO_Pin_12, LOW};
    Buzzer_Init(RCC_APB2Periph_GPIOB, &buzzer);

    Key buzzer_key = {GPIOB, GPIO_Pin_1, LOW};
    Key_Init(RCC_APB2Periph_GPIOB, &buzzer_key);

    for (;;) {
        if (Key_Read(&led_key)) {
            LED_Turn(&led);
        }
        if (Key_Read(&buzzer_key)) {
            Buzzer_Turn(&buzzer);
        }
    }
}
