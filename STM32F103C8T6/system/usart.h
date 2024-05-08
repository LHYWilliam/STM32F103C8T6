#ifndef __USART_H_
#define __USART_H_

#include "gpio.h"

typedef struct {
    GPIO *TX;
    GPIO *RX;

    uint32_t RCC_APBPeriph;
    USART_TypeDef *USARTx;
    uint16_t USART_Mode;

} USART;

void USART_Init_(USART *usart);
void USART_SendByte(USART *usart, uint8_t byte);
void USART_SendHex(USART *usart, uint8_t byte);
void USART_SendString(USART *usart, char *format, ...);

#endif