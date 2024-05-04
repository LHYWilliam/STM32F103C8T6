#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "contrast_sensor.h"
#include "oled.h"

extern uint16_t counter;

void EXTI15_10_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line13) == SET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13)) {
            counter++;
        }
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
}

int main() {
    OLED_Init();

    ContrastSensor sensor = {
        GPIOB,
        GPIO_Pin_13,
    };
    GPIO_EXIT_Interrut interrupt = {
        GPIO_PortSourceGPIOB,
        GPIO_PinSource13,
        EXTI_Line13,
        EXTI_Trigger_Falling,
        EXTI15_10_IRQn,
        NVIC_PriorityGroup_2,
        1,
        1,
    };
    ContrastSensor_Init(RCC_APB2Periph_GPIOB, &sensor, &interrupt);

    for (;;) {
        OLED_ShowNum(1, 1, ContrastSensor_Get(), 3);
    }
}
