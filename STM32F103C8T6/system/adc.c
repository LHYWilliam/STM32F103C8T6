#include "stm32f10x.h"

#include <string.h>

#include "adc.h"
#include "gpio.h"

void ADC_Init_(ADC *adc) {
    GPIO gpio = {
        .Mode = GPIO_Mode_AIN,
    };
    strcpy(gpio.GPIOxPiny, adc->gpio);
    GPIO_Init_(&gpio);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADCx(adc->ADCx), ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    uint8_t NbrOfChannel = 1;
    char *temp = adc->channel;
    do {
        ADC_RegularChannelConfig(adc->ADCx, ADC_Channel_x(temp), NbrOfChannel,
                                 ADC_SampleTime_55Cycles5);
    } while ((temp = strchr(temp, '|'), temp) && (temp = temp + 2) &&
             (NbrOfChannel = NbrOfChannel + 1));

    ADC_InitTypeDef ADC_InitTStruct = {
        .ADC_Mode = ADC_Mode_Independent,
        .ADC_NbrOfChannel = NbrOfChannel,
        .ADC_DataAlign = ADC_DataAlign_Right,
        .ADC_ExternalTrigConv = ADC_ExternalTrigConv_None,
        .ADC_ContinuousConvMode = ENABLE,
        .ADC_ScanConvMode = NbrOfChannel > 1 ? ENABLE : DISABLE,
    };
    ADC_Init(adc->ADCx, &ADC_InitTStruct);
}

void ADC_Start(ADC *adc) {
    if (adc->DMA) {
        ADC_DMACmd(adc->ADCx, ENABLE);
    }

    ADC_Cmd(adc->ADCx, ENABLE);

    ADC_ResetCalibration(adc->ADCx);
    while (ADC_GetResetCalibrationStatus(adc->ADCx) == SET)
        ;
    ADC_StartCalibration(adc->ADCx);
    while (ADC_GetCalibrationStatus(adc->ADCx) == SET)
        ;

    ADC_SoftwareStartConvCmd(adc->ADCx, ENABLE);
}

uint16_t ADC_Get(ADC *adc) {
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
        ;

    return ADC_GetConversionValue(adc->ADCx);
}