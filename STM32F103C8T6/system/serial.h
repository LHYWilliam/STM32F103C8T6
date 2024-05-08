#ifndef __SERIAL_H_
#define __SERIAL_H_

#include "gpio.h"

typedef struct {
    GPIO *TX;
    GPIO *RX;

    uint32_t RCC_APBPeriph;
    USART_TypeDef *USARTx;
    uint16_t USART_Mode;
} Serial;

void Serial_Init(Serial *serial);
void Serial_SendByte(Serial *serial, uint8_t byte);
void Serial_SendString(Serial *serial, char *string);
void Serial_Send(Serial *serial, char *format, ...);

#endif