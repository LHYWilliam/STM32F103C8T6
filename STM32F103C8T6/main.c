#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "contrast_sensor.h"
#include "interrupt.h"
#include "key.h"
#include "led.h"
#include "oled.h"

uint16_t counter;

void EXTI15_10_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line12) == SET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
            counter++;
        }
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}

int main() {
    OLED_Init();

    ContrastSensor sensor = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_12,
    };
    ContrastSensor_Init(&sensor);

    GPIO_EXTIInterrut interrupt = {
        GPIO_PortSourceGPIOB,
        GPIO_PinSource12,
        EXTI_Line12,
        EXTI_Trigger_Falling,
        EXTI15_10_IRQn,
        NVIC_PriorityGroup_2,
        1,
        1,
    };
    GPIO_EXTIInterrut_Init(&interrupt);

    for (;;) {
        OLED_ShowNum(1, 1, counter, 3);
    }
}
