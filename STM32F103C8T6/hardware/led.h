#ifndef __LED_H
#define __LED_H

#include "gpio.h"

#define LOW ((uint8_t)0)
#define HIGH ((uint8_t)01)

typedef struct {
    GPIO *gpio;
    uint8_t Mode;
} LED;

void LED_Init(LED *led);
void LED_On(LED *led);
void LED_Off(LED *led);
void LED_Turn(LED *led);

#endif