#ifndef __USART_H_
#define __USART_H_

#include "stm32f10x.h"

typedef struct {
    uint32_t RCC_APBPeriph;
    USART_TypeDef *USARTx;
    uint16_t USART_Mode;

} USART;

void USART_Init_(USART *usart);

#endif