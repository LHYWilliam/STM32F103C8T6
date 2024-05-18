#ifndef __MPUIIC_H
#define __MPUIIC_H

#include "stdint.h"

#define MPU_SDA_IN()                                                           \
    {                                                                          \
        GPIOB->CRH &= 0XFFFF0FFF;                                              \
        GPIOB->CRH |= 8 << 12;                                                 \
    }
#define MPU_SDA_OUT()                                                          \
    {                                                                          \
        GPIOB->CRH &= 0XFFFF0FFF;                                              \
        GPIOB->CRH |= 3 << 12;                                                 \
    }

#define GPIOB_IDR_Addr (GPIOB_BASE + 8)
#define GPIOB_ODR_Addr (GPIOB_BASE + 12)

#define BITBAND(addr, bitnum)                                                  \
    ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n)
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)

#define MPU_IIC_SCL PBout(10)
#define MPU_IIC_SDA PBout(11)
#define MPU_READ_SDA PBin(11)

#define MPU_IIC_SCL PBout(10)
#define MPU_IIC_SDA PBout(11)
#define MPU_READ_SDA PBin(11)

void I2C_Delay(void);
void I2C_Init_(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t txd);
uint8_t I2C_ReceiveByte(unsigned char ack);
uint8_t I2C_WaitAck(void);
void I2C_Ack(void);
void I2C_NoAck(void);

uint8_t I2C_Send(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len);
uint8_t I2C_Receive(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
#endif