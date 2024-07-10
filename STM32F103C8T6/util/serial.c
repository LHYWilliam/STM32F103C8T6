#include "stdarg.h"
#include "stdio.h"
#include <stdint.h>
#include <string.h>

#include "gpio.h"
#include "serial.h"

void Serial_Init(Serial *serial) {
    if (serial->TX[0]) {
        GPIO TX = {
            .GPIO_Mode = GPIO_Mode_AF_PP,
        };
        strcpy(TX.GPIOxPiny, serial->TX);
        GPIO_Init_(&TX);
    }
    if (serial->RX[0]) {
        GPIO RX = {
            .GPIO_Mode = GPIO_Mode_IPU,
        };
        strcpy(RX.GPIOxPiny, serial->RX);
        GPIO_Init_(&RX);
    }

    USART_Init_(serial->usart);

    serial->count = 0;
    serial->RecieveFlag = RESET;
    serial->type = None;
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

void Serial_SendHexPack(Serial *serial, uint8_t *array, uint16_t length) {
    Serial_SendByte(serial, 0xFF);
    for (uint8_t i = 0; i < length; i++) {
        Serial_SendByte(serial, array[i]);
    }
    Serial_SendByte(serial, 0xFE);
}

void Serial_SendStringPack(Serial *serial, char *string) {
    Serial_SendString(serial, ">%s\r\n", string);
}

void Serial_Parse(Serial *serial) {
    serial->ByteData = USART_ReceiveData(serial->usart->USARTx);

    switch (serial->type) {
    case None:

        if (serial->ByteData == 0xFF) {
            serial->type = HexPack;
        } else if (serial->ByteData == '>') {
            serial->type = StringPack;
        } else {
            serial->type = Byte;
            serial->RecieveFlag = SET;
        }
        break;

    case HexPack:
        if (serial->ByteData == 0xFE) {
            serial->RecieveFlag = SET;
        } else {
            serial->HexData[serial->count++] = serial->ByteData;
        }
        break;

    case StringPack:
        // if (serial->count >= 1 &&
        //     serial->StringData[serial->count - 1] == '\r' &&
        //     serial->ByteData == '\n') {
        //     serial->StringData[serial->count - 1] = '\0';
        if (serial->count >= 1 && serial->ByteData == '\r') {
            serial->StringData[serial->count] = '\0';
            serial->RecieveFlag = SET;
        } else if (serial->ByteData == 0x08) {
            serial->count--;
        } else {
            serial->StringData[serial->count++] = serial->ByteData;
        }
        break;

    default:
        break;
    }
}

void Serial_Clear(Serial *serial) {
    serial->count = 0;
    serial->type = None;
    serial->RecieveFlag = RESET;
}