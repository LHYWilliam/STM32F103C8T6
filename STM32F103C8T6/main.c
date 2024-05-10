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

uint8_t count = 0;
uint8_t RecieveFlag = RESET;

PackType type = None;

uint8_t byte;
uint8_t HexData[32];
char StringData[32];

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
    Serial_Init(&serial);

    USART_Interrupt interrupt = {
        USART1, USART_IT_RXNE, USART1_IRQn, NVIC_PriorityGroup_2, 1, 1,
    };
    USART_Interrupt_Init(&interrupt);

    uint8_t array[] = {0x48, 0x49, 0x50, 0x51};
    for (;;) {
        if (Key_Read(&key)) {
            // Serial_SendHexPack(&serial, array, 4);
            Serial_SendStringPack(&serial, "hello world!");
        };
        if (RecieveFlag == SET) {
            if (type == ByteData) {
                OLED_ShowHexNum(1, 1, byte, 2);
            } else if (type == HexPack) {
                for (uint8_t i = 0; i < count; i++) {
                    OLED_ShowHexNum(1, 1 + 3 * i, HexData[i], 2);
                }
            } else if (type == StringPack) {
                OLED_ShowString(1, 1, StringData);
            }

            count = 0;
            type = None;
            RecieveFlag = RESET;
        }
    }
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
        byte = USART_ReceiveData(USART1);

        if (type == None) {
            if (byte == 0xFF) {
                type = HexPack;
            } else if (byte == '>') {
                type = StringPack;
            } else {
                type = ByteData;
                RecieveFlag = SET;
            }
        } else if (type == HexPack) {
            if (byte == 0xFE) {
                RecieveFlag = SET;
            } else {
                HexData[count++] = byte;
            }
        } else if (type == StringPack) {
            if (count >= 1 && byte == '\n' && StringData[count - 1] == '\r') {
                StringData[--count] = '\0';

                RecieveFlag = SET;
            } else {
                StringData[count++] = byte;
            }
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}