#include "servo.h"
#include "pwm.h"

void Servo_Init(Servo *servo) { PWM_Init(servo->pwm); }

void Servo_SetAngel(Servo *servo, float angel) {
    PWM_SetPulse(servo->pwm, angel / 180 * 2000 + 500);
}