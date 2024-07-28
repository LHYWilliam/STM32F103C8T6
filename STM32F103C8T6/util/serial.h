#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"

#define USARTx_IRQn(x)                                                         \
    ((x) == USART1   ? USART1_IRQn                                             \
     : (x) == USART2 ? USART2_IRQn                                             \
     : (x) == USART3 ? USART3_IRQn                                             \
                     : NULL)

typedef enum {
    None,
    Byte,
    HexPack,
    StringPack,
} PackType;

typedef struct {
    char TX[4];
    char RX[4];
    USART_TypeDef *USARTx;

    uint8_t Interrupt;
    uint8_t DMA;

    uint8_t count;
    uint8_t RecieveFlag;

    PackType type;

    uint8_t ByteData;
    uint8_t HexData[32];
    char StringData[32];
} Serial;

void Serial_Init(Serial *serial);

void Serial_SendByte(Serial *serial, uint8_t byte);
void Serial_SendHex(Serial *serial, uint8_t byte);
void Serial_SendString(Serial *serial, char *format, ...);

void Serial_SendHexPack(Serial *serial, uint8_t *array, uint16_t length);
void Serial_SendStringPack(Serial *serial, char *string);

void Serial_Parse(Serial *serial);

void Serial_Clear(Serial *serial);

#endif