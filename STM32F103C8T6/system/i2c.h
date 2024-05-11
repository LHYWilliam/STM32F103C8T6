#ifndef __I2C_H
#define __I2C_H

#include "stm32f10x.h"

typedef struct {
    uint32_t RCC_APB1Periph;
    I2C_TypeDef *I2Cx;

    uint32_t I2C_ClockSpeed;
} I2C;

void I2C_Init_(I2C *i2c);
void I2C_Send(I2C *i2c, uint8_t DeviceAddress, uint8_t RegisterAddress,
              const uint8_t *bytes, uint8_t length);
void I2C_Receive(I2C *i2c, uint8_t DeviceAddress, uint8_t RegisterAddress,
                 uint8_t *bytes, uint8_t length);

void I2C_WaitEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT);

#endif