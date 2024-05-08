#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include <stdlib.h>

#include "gpio.h"
#include "key.h"
#include "oled.h"
#include "serial.h"

int main() {
    OLED_Init();
    GPIO gpio_key = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_11,
        GPIO_Mode_IPU,
    };
    Key key = {
        &gpio_key,
        LOW,
    };
    Key_Init(&key);

    GPIO gpio_serial = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_9,
        GPIO_Mode_AF_PP,
    };
    Serial serial = {
        &gpio_serial, NULL, RCC_APB2Periph_USART1, USART1, USART_Mode_Tx,
    };
    Serial_Init(&serial);

    for (;;) {
        if (Key_Read(&key)) {
            Serial_Send(&serial, "%lf\r\n", 114.514);
        }
    }
}