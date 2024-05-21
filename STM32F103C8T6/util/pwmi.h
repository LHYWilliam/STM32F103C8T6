#ifndef __PWMI_H
#define __PWMI_H

#include "capture.h"
#include "gpio.h"
#include "tim.h"

typedef struct {
    GPIO *gpio;
    Capture *frequency;
    Capture *duty;
    TIM *tim;
} PWMI;

void PWMI_Init(PWMI *pwmi);

uint16_t PWMI_GetFrequency(PWMI *pwmi);
uint16_t PWMI_GetDuty(PWMI *pwmi);

#endif