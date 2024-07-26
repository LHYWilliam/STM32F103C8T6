#include <stdint.h>
#include <string.h>

#include "compare.h"
#include "gpio.h"
#include "servo.h"
#include "tim.h"

void Servo_Init(Servo *servo) {
    GPIO PWM = {
        .GPIO_Mode = GPIO_Mode_AF_PP,
    };
    strcpy(PWM.GPIOxPiny, servo->PWM);
    GPIO_Init_(&PWM);

    TIM tim = {
        .RCC_APBxPeriph = RCC_APBxPeriph_TIMx(servo->TIMx),
        .TIMx = servo->TIMx,
        .TIM_ClockSource = TIM_InternalClock,
        .TIM_Prescaler = 72 - 1,
        .TIM_Period = 20000 - 1,
        .CMD_Mode = CMD,
    };
    TIM_Init(&tim, NULL);

    uint8_t count = 0;
    char *temp = servo->channel;
    do {
        Compare compare = {
            .TIMx = servo->TIMx,
            .TIM_Pulse = 0,
            .TIM_OCInit = TIM_OCxInit(temp[0] - '0'),
            .TIM_SetCompare = TIM_SetComparex(temp[0] - '0'),
        };
        Compare_Init(&compare);
        servo->TIM_SetCompare[count] = compare.TIM_SetCompare;
    } while ((temp = strchr(temp, '|'), temp) && (temp = temp + 2) &&
             (count = count + 1));
}

void Servo_set(Servo *servo, float angle1, float angle2) {
    servo->TIM_SetCompare[0](servo->TIMx, angle1 / 180 * 2000 + 500);
    servo->TIM_SetCompare[1](servo->TIMx, angle2 / 180 * 2000 + 500);
}