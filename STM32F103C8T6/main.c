#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "gpio.h"
#include "key.h"
#include "oled.h"
#include "usart.h"

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

    GPIO gpio_TX = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_9,
        GPIO_Mode_AF_PP,
    };
    GPIO gpio_RX = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_10,
        GPIO_Mode_IPU,
    };
    USART usart = {
        &gpio_TX,
        &gpio_RX,
        RCC_APB2Periph_USART1,
        USART1,
        USART_Mode_Tx | USART_Mode_Rx,
    };
    USART_Init_(&usart);

    uint16_t data;
    for (;;) {
        if (Key_Read(&key)) {
            USART_SendString(&usart, "%s", "你好世界\r\n");
            // Serial_SendHex(&serial, 0x55);
        };
        if (USART_GetFlagStatus(usart.USARTx, USART_FLAG_RXNE) == SET) {
            OLED_ShowHexNum(1, 1, USART_ReceiveData(usart.USARTx), 2);
        };
    }
}