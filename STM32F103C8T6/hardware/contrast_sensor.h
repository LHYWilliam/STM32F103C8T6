#ifndef __CONTRAST_SENSOR_H
#define __CONTRAST_SENSOR_H

#include "stm32f10x.h"

typedef struct {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint8_t Mode;
} ContrastSensor;

typedef struct {
    uint8_t GPIO_PortSource;
    uint8_t GPIO_PinSource;
    uint32_t EXTI_Line;
    EXTITrigger_TypeDef EXTI_Trigger;
    uint8_t NVIC_IRQChannel;
    uint32_t NVIC_PriorityGroup;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;

} GPIO_EXIT_Interrut;

void ContrastSensor_Init(uint32_t RCC_APB2Periph,
                         ContrastSensor *contrast_sensor,
                         GPIO_EXIT_Interrut *interrupt);
uint16_t ContrastSensor_Get();

#endif