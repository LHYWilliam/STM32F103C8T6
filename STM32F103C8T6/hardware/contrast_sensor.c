#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "contrast_sensor.h"

uint16_t counter;

void ContrastSensor_Init(uint32_t RCC_APB2Periph,
                         ContrastSensor *contrast_sensor,
                         GPIO_EXIT_Interrut *interrupt) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {
        contrast_sensor->GPIO_Pin,
        GPIO_Speed_50MHz,
        GPIO_Mode_IPU,
    };
    GPIO_Init(contrast_sensor->GPIOx, &GPIO_InitStruct);

    if (interrupt) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        GPIO_EXTILineConfig(interrupt->GPIO_PortSource,
                            interrupt->GPIO_PinSource);

        EXTI_InitTypeDef EXTI_InitStruct = {
            interrupt->EXTI_Line,
            EXTI_Mode_Interrupt,
            interrupt->EXTI_Trigger,
            ENABLE,
        };
        EXTI_Init(&EXTI_InitStruct);

        NVIC_PriorityGroupConfig(interrupt->NVIC_PriorityGroup);
        NVIC_InitTypeDef NVIC_InitStruct = {
            interrupt->NVIC_IRQChannel,
            interrupt->NVIC_IRQChannelPreemptionPriority,
            interrupt->NVIC_IRQChannelSubPriority,
            ENABLE,
        };
        NVIC_Init(&NVIC_InitStruct);
    }
}

uint16_t ContrastSensor_Get() { return counter; }