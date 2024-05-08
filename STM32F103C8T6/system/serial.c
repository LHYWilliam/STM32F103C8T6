#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "stdarg.h"
#include "stdio.h"

#include "serial.h"

void Serial_Init(Serial *serial) {
    if (serial->TX) {
        GPIO_Init_(serial->TX);
    }
    if (serial->RX) {
        GPIO_Init_(serial->RX);
    }

    if (serial->RCC_APBPeriph == RCC_APB2Periph_USART1) {
        RCC_APB2PeriphClockCmd(serial->RCC_APBPeriph, ENABLE);
    } else {
        RCC_APB1PeriphClockCmd(serial->RCC_APBPeriph, ENABLE);
    }

    USART_InitTypeDef USART_InitStruct = {
        9600,
        USART_WordLength_8b,
        USART_StopBits_1,
        USART_Parity_No,
        serial->USART_Mode,
        USART_HardwareFlowControl_None,
    };
    USART_Init(serial->USARTx, &USART_InitStruct);

    USART_Cmd(serial->USARTx, ENABLE);
}

void Serial_SendByte(Serial *serial, uint8_t byte) {
    while (USART_GetFlagStatus(serial->USARTx, USART_FLAG_TXE) == RESET)
        ;
    USART_SendData(serial->USARTx, byte);
}

void Serial_SendString(Serial *serial, char *string) {
    for (uint8_t i = 0; string[i] != '\0'; i++) {
        Serial_SendByte(serial, string[i]);
    }
}

void Serial_Send(Serial *serial, char *format, ...) {
    char string[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(string, format, arg);
    va_end(arg);
    Serial_SendString(serial, string);
}