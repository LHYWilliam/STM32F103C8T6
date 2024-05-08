#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "gpio.h"
#include "interrupt.h"
#include "key.h"
#include "oled.h"
#include "serial.h"
#include "usart.h"
#include <stdint.h>

Serial *serial_it;
uint8_t data;

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
        RCC_APB2Periph_USART1,
        USART1,
        USART_Mode_Tx | USART_Mode_Rx,
    };
    Serial serial = {
        &gpio_TX,
        &gpio_RX,
        &usart,
    };
    serial_it = &serial;
    Serial_Init(&serial);

    USART_Interrupt interrupt = {
        USART1, USART_IT_RXNE, USART1_IRQn, NVIC_PriorityGroup_2, 1, 1,
    };
    USART_Interrupt_Init(&interrupt);

    uint8_t array[] = {0x48, 0x49, 0x50, 0x51};
    for (;;) {
        if (Key_Read(&key)) {
            Serial_SendHexPack(&serial, array, 4);
            // Serial_SendStringPack(&serial, "hello world!");
        };
        if (serial.RecieveState == GET) {
            OLED_ShowChar(1, 1, data);
            serial.RecieveState = WAIT;
        }
    }
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
        data = USART_ReceiveData(USART1);
        serial_it->RecieveState = GET;

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}