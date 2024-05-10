#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"

#include "adc.h"

void ADC_Init_(ADC *adc) {
    GPIO_Init_(adc->gpio);

    RCC_APB2PeriphClockCmd(adc->RCC_APB2Periph, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    for (uint8_t i = 0; i < adc->ADC_NbrOfChannel; i++) {
        ADC_RegularChannelConfig(adc->ADCx, adc->ADC_Channel[i], i + 1,
                                 ADC_SampleTime_55Cycles5);
    }

    ADC_InitTypeDef ADC_InitTStruct = {
        ADC_Mode_Independent,
        adc->ADC_NbrOfChannel > 1 ? ENABLE : DISABLE,
        adc->ADC_ContinuousConvMode,
        adc->ADC_ExternalTrigConv,
        ADC_DataAlign_Right,
        adc->ADC_NbrOfChannel,
    };
    ADC_Init(adc->ADCx, &ADC_InitTStruct);

    ADC_DMACmd(adc->ADCx, adc->DMA_Mode);

    ADC_Cmd(adc->ADCx, ENABLE);

    ADC_ResetCalibration(adc->ADCx);
    while (ADC_GetResetCalibrationStatus(adc->ADCx) == SET)
        ;
    ADC_StartCalibration(adc->ADCx);
    while (ADC_GetCalibrationStatus(adc->ADCx) == SET)
        ;
}

uint16_t ADC_Get(ADC *adc) {

    while (adc->ADC_ContinuousConvMode == DISABLE &&
           ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
        ;

    return ADC_GetConversionValue(adc->ADCx);
}