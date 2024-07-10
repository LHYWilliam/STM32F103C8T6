#ifndef __I2C_H
#define __I2C_H

#include "stdint.h"

#include "gpio.h"

#define SDA_IN()                                                               \
    {                                                                          \
        GPIOB->CRH &= 0XFFFFFF0F;                                              \
        GPIOB->CRH |= 8 << 4;                                                  \
    }
#define SDA_OUT()                                                              \
    {                                                                          \
        GPIOB->CRH &= 0XFFFFFF0F;                                              \
        GPIOB->CRH |= 3 << 4;                                                  \
    }

#define I2C_SCL PBout(8)
#define I2C_SDA PBout(9)
#define I2C_READ_SDA PBin(9)

typedef struct {
    char SCL[4];
    char SDA[4];
} I2C;

void I2C_Init_(I2C *i2c);

void I2C_Start(I2C *i2c);
void I2C_Stop(I2C *i2c);

void I2C_SendByte(I2C *i2c, uint8_t txd);
uint8_t I2C_ReceiveByte(I2C *i2c, unsigned char ack);

uint8_t I2C_WaitAck(I2C *i2c);
void I2C_Ack(I2C *i2c);
void I2C_NoAck(I2C *i2c);

void I2C_Delay(I2C *i2c);

uint8_t I2C_Send(I2C *i2c, uint8_t addr, uint8_t reg, const uint8_t *buf,
                 uint8_t len);
uint8_t I2C_Receive(I2C *i2c, uint8_t addr, uint8_t reg, uint8_t *buf,
                    uint8_t len);
#endif