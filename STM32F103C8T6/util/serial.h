#ifndef __SERIAL_H
#define __SERIAL_H

#include "gpio.h"
#include "usart.h"

#define WAIT ((uint8_t)0)
#define GET ((uint8_t)1)

typedef struct {
    GPIO *TX;
    GPIO *RX;
    USART *usart;
    uint8_t RecieveState;
} Serial;

void Serial_Init(Serial *serial);
void Serial_SendByte(Serial *serial, uint8_t byte);
void Serial_SendHex(Serial *serial, uint8_t byte);
void Serial_SendString(Serial *serial, char *format, ...);

#endif