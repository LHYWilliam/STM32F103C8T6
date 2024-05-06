#include "stm32f10x_tim.h"

#include "compare.h"

void Compare_Init(Compare *compare) {
    TIM_OCInitTypeDef TIM_OCInitStruct = {
        .TIM_OCMode = TIM_OCMode_PWM1,
        .TIM_OCPolarity = TIM_OCPolarity_High,
        .TIM_OutputState = TIM_OutputState_Enable,
        .TIM_Pulse = compare->TIM_Pulse,
    };
    compare->TIM_OCInit(compare->TIMx, &TIM_OCInitStruct);
}

void Compare_Set(Compare *compare, uint16_t val) {
    compare->TIM_SetCompare(compare->TIMx, val);
}