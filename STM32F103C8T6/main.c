#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include <stdint.h>

#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "oled.h"

int main() {
    OLED_Init();
    uint16_t ADValue[2];
    float Voltage[2];

    GPIO gpio = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_0 | GPIO_Pin_1,
        GPIO_Mode_AIN,
    };
    ADC_Channels channels = {ADC_Channel_0, ADC_Channel_1};
    ADC adc = {
        &gpio,  RCC_APB2Periph_ADC1,       ADC1,   2, channels,
        ENABLE, ADC_ExternalTrigConv_None, ENABLE,
    };
    ADC_Init_(&adc);

    DMA dma = {
        RCC_AHBPeriph_DMA1,
        DMA1_Channel1,
        (uint32_t)&ADC1->DR,
        DMA_PeripheralDataSize_HalfWord,
        DMA_PeripheralInc_Disable,
        (uint32_t)ADValue,
        DMA_MemoryDataSize_HalfWord,
        DMA_MemoryInc_Enable,
        2,
        DMA_DIR_PeripheralSRC,
        DMA_Mode_Circular,
        DMA_Priority_Medium,
        DMA_M2M_Disable,
    };
    DMA_Init_(&dma);

    ADC_SoftwareStartConvCmd(adc.ADCx, ENABLE);
    DMA_Enable(&dma);

    OLED_ShowString(1, 1, "ADValue:");
    OLED_ShowString(2, 1, "Voltage:0.00V");
    OLED_ShowString(3, 1, "ADValue:");
    OLED_ShowString(4, 1, "Voltage:0.00V");

    for (;;) {
        for (int i = 0; i < 2; i++) {
            Voltage[i] = (float)ADValue[i] / 4095 * 3.3;
            OLED_ShowNum(1 + i * 2, 9, ADValue[i], 4);
            OLED_ShowNum(2 + i * 2, 9, Voltage[i], 1);
            OLED_ShowNum(2 + i * 2, 11, (uint16_t)(Voltage[i] * 100) % 100, 2);
        }
    }
}
