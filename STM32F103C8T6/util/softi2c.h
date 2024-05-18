#ifndef __MPUIIC_H
#define __MPUIIC_H

#include <stdint.h>

// IO方向设置
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

#define BITBAND(addr, bitnum)                                                  \
    ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

#define GPIOB_ODR_Addr (GPIOB_BASE + 12) // 0x40010C0C
#define GPIOB_IDR_Addr (GPIOB_BASE + 8)  // 0x40010C08

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) // 输出
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  // 输入

// IO操作函数
#define MPU_IIC_SCL PBout(10) // SCL
#define MPU_IIC_SDA PBout(11) // SDA
#define MPU_READ_SDA PBin(11) // 输入SDA

#define MPU_ADDR 0X68

// IIC所有操作函数
void MPU_IIC_Delay(void);                     // MPU IIC延时函数
void MPU_IIC_Init(void);                      // 初始化IIC的IO口
void MPU_IIC_Start(void);                     // 发送IIC开始信号
void MPU_IIC_Stop(void);                      // 发送IIC停止信号
void MPU_IIC_Send_Byte(uint8_t txd);          // IIC发送一个字节
uint8_t MPU_IIC_Read_Byte(unsigned char ack); // IIC读取一个字节
uint8_t MPU_IIC_Wait_Ack(void);               // IIC等待ACK信号
void MPU_IIC_Ack(void);                       // IIC发送ACK信号
void MPU_IIC_NAck(void);                      // IIC不发送ACK信号

void IMPU_IC_Write_One_Byte(uint8_t daddr, uint8_t addr, uint8_t data);
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr, uint8_t addr);
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len,
                      uint8_t *buf); // IIC连续写
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len,
                     uint8_t *buf);                // IIC连续读
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data); // IIC写一个字节
uint8_t MPU_Read_Byte(uint8_t reg);                // IIC读一个字节

#endif
