#include "servo.h"
#include "pwm.h"

void Servo_Init(Servo *servo) { PWM_Init(servo->pwm); }

void Servo_Set(Servo *servo, float angel) {
    servo->pwm->compare->TIM_SetCompare(servo->pwm->tim->TIMx,
                                        angel / 180 * 2000 + 500);
}