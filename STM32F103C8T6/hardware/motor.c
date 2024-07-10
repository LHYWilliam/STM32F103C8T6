#include "stm32f10x_gpio.h"

#include <string.h>

#include "gpio.h"
#include "motor.h"
#include "pwm.h"

void Motor_Init(Motor *motor) {
    PWM_Init(motor->pwm);

    GPIO direction1 = {
        .GPIO_Mode = GPIO_Mode_Out_PP,
    };
    strcpy(direction1.GPIOxPiny, motor->direction1);
    GPIO_Init_(&direction1);

    GPIO direction2 = {
        .GPIO_Mode = GPIO_Mode_Out_PP,
    };
    strcpy(direction2.GPIOxPiny, motor->direction2);
    GPIO_Init_(&direction2);
}
void Motor_Set(Motor *motor, int16_t speed) {
    GPIO_WriteBit(GPIOx(motor->direction1), GPIO_Pinx(motor->direction1),
                  speed >= 0 ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(GPIOx(motor->direction2), GPIO_Pinx(motor->direction2),
                  speed >= 0 ? Bit_SET : Bit_RESET);
    PWM_SetPulse(motor->pwm, speed >= 0 ? speed : -speed);
}