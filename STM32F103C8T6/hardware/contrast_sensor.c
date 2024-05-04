#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stdint.h>

#include "contrast_sensor.h"

uint16_t counter;

void ContrastSensor_Init(uint32_t RCC_APB2Periph,
                         ContrastSensor *contrast_sensor) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {contrast_sensor->GPIO_Pin,
                                        GPIO_Speed_50MHz, GPIO_Mode_IPU};
    GPIO_Init(contrast_sensor->GPIOx, &GPIO_InitStruct);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);

    EXTI_InitTypeDef EXTI_InitStruct = {EXTI_Line13, EXTI_Mode_Interrupt,
                                        EXTI_Trigger_Falling, ENABLE};
    EXTI_Init(&EXTI_InitStruct);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStruct = {EXTI15_10_IRQn, 1, 1, ENABLE};
    NVIC_Init(&NVIC_InitStruct);
}

void EXTI15_10_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line13) == SET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13)) {
            counter++;
        }
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
}

uint16_t ContrastSensor_Get() { return counter; }