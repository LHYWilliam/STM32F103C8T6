#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "stdarg.h"
#include "stdio.h"

#include "usart.h"

void USART_Init_(USART *usart) {
    if (usart->TX) {
        GPIO_Init_(usart->TX);
    }
    if (usart->RX) {
        GPIO_Init_(usart->RX);
    }

    if (usart->RCC_APBPeriph == RCC_APB2Periph_USART1) {
        RCC_APB2PeriphClockCmd(usart->RCC_APBPeriph, ENABLE);
    } else {
        RCC_APB1PeriphClockCmd(usart->RCC_APBPeriph, ENABLE);
    }

    USART_InitTypeDef USART_InitStruct = {
        9600,
        USART_WordLength_8b,
        USART_StopBits_1,
        USART_Parity_No,
        usart->USART_Mode,
        USART_HardwareFlowControl_None,
    };
    USART_Init(usart->USARTx, &USART_InitStruct);

    // USART_ITConfig(usart->USARTx, USART_IT_RXNE, ENABLE);

    USART_Cmd(usart->USARTx, ENABLE);
}

void USART_SendByte(USART *usart, uint8_t byte) {
    while (USART_GetFlagStatus(usart->USARTx, USART_FLAG_TXE) == RESET)
        ;
    USART_SendData(usart->USARTx, byte);
}

void USART_SendHex(USART *usart, uint8_t byte) { USART_SendByte(usart, byte); }

void USART_SendString(USART *usart, char *format, ...) {
    char string[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(string, format, arg);
    va_end(arg);
    for (uint8_t i = 0; string[i] != '\0'; i++) {
        USART_SendByte(usart, string[i]);
    }
}