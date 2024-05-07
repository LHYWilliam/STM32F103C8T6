#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include <stdint.h>

#include "adc.h"
#include "gpio.h"
#include "oled.h"

int16_t speed;

int main() {
    OLED_Init();

    GPIO gpio = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_0,
        GPIO_Mode_AIN,
    };
    GPIOs gpios = {&gpio};
    ADC_Channels channels = {ADC_Channel_0};
    ADC adc = {
        gpios,  RCC_APB2Periph_ADC1,       ADC1, 1, channels,
        ENABLE, ADC_ExternalTrigConv_None,
    };
    ADC_Init_(&adc);
    ADC_SoftwareStartConvCmd(adc.ADCx, ENABLE);

    OLED_ShowString(1, 1, "ADValue:");
    OLED_ShowString(2, 1, "Voltage:0.00V");

    uint16_t ADValue;
    float Voltage;
    for (;;) {
        ADValue = ADC_Get(&adc);
        Voltage = (float)ADValue / 4095 * 3.3;

        OLED_ShowNum(1, 9, ADValue, 4);
        OLED_ShowNum(2, 9, Voltage, 1);
        OLED_ShowNum(2, 11, (uint16_t)(Voltage * 100) % 100, 2);
    }
}
