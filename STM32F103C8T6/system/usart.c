#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "usart.h"

void USART_Init_(USART *usart) {
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