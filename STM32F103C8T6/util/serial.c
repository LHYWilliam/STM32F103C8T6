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

    USART_Init_(serial->usart);

    serial->RecieveState = WAIT;
}

void Serial_SendByte(Serial *serial, uint8_t byte) {
    while (USART_GetFlagStatus(serial->usart->USARTx, USART_FLAG_TXE) == RESET)
        ;
    USART_SendData(serial->usart->USARTx, byte);
}

void Serial_SendHex(Serial *serial, uint8_t byte) {
    Serial_SendByte(serial, byte);
}

void Serial_SendString(Serial *serial, char *format, ...) {
    char string[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(string, format, arg);
    va_end(arg);
    for (uint8_t i = 0; string[i] != '\0'; i++) {
        Serial_SendByte(serial, string[i]);
    }
}