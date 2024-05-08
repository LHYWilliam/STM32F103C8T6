#ifndef __SERIAL_H
#define __SERIAL_H

#include "gpio.h"
#include "usart.h"

typedef struct {
    GPIO *TX;
    GPIO *RX;
    USART *usart;

} Serial;

void Serial_Init(Serial *serial);
void Serial_SendByte(Serial *serial, uint8_t byte);
void Serial_SendHex(Serial *serial, uint8_t byte);
void Serial_SendString(Serial *serial, char *format, ...);

#endif