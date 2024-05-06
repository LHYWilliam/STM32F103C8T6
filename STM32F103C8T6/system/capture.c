#include "stm32f10x_tim.h"

#include "capture.h"

void Capture_Init(Capture *capture) {
    TIM_ICInitTypeDef TIM_ICInitStruct = {
        capture->TIM_Channel, capture->TIM_ICPolarity, capture->TIM_ICSelection,
        TIM_ICPSC_DIV1,       capture->TIM_ICFilter,
    };
    TIM_ICInit(capture->TIMx, &TIM_ICInitStruct);
}

uint16_t Capture_GetFreq(Capture *capture) {
    return 1000000 / (capture->TIM_GetCapture(capture->TIMx) + 1);
}

uint16_t Capture_GetDuty(Capture *freq, Capture *duty) {
    return (duty->TIM_GetCapture(duty->TIMx) + 1) * 100 /
           (freq->TIM_GetCapture(freq->TIMx) + 1);
}