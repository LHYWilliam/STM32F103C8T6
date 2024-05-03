#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "led.h"

void LED_Init(uint32_t RCC_APB2Periph, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
              GPIOSpeed_TypeDef GPIO_Speed, GPIOMode_TypeDef GPIO_Mode) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {GPIO_Pin, GPIO_Speed, GPIO_Mode};
    GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void LED_On(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_WriteBit(GPIOx, GPIO_Pin, Bit_SET);
}

void LED_Off(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_WriteBit(GPIOx, GPIO_Pin, Bit_RESET);
}

void LED_Turn(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint8_t now = GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin);
    GPIO_WriteBit(GPIOx, GPIO_Pin, (BitAction)!now);
}