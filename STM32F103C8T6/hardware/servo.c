#include <stdlib.h>

#include "servo.h"

void Servo_Init(Servo *servo) {
    TIM_Init(servo->tim, NULL);
    Compare_Init(servo->compare);
    GPIO_Init_(servo->gpio);
}

void Servo_Set(Servo *servo, float angel) {
    servo->compare->TIM_SetCompare(servo->tim->TIMx, angel / 180 * 2000 + 500);
}