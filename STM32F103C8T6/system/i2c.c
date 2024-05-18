#include "stm32f10x.h"

#include "delay.h"
#include "i2c.h"

void I2C_Delay(void) { Delay_us(2); }

void I2C_Init_(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
}
// 产生IIC起始信号
void I2C_Start(void) {
    MPU_SDA_OUT();
    MPU_IIC_SDA = 1;
    MPU_IIC_SCL = 1;
    I2C_Delay();
    MPU_IIC_SDA = 0;
    I2C_Delay();
    MPU_IIC_SCL = 0;
}
// 产生IIC停止信号
void I2C_Stop(void) {
    MPU_SDA_OUT();
    MPU_IIC_SCL = 0;
    MPU_IIC_SDA = 0;
    I2C_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_SDA = 1;
    I2C_Delay();
}

uint8_t I2C_WaitAck(void) {
    uint8_t ucErrTime = 0;
    MPU_SDA_IN();
    MPU_IIC_SDA = 1;
    I2C_Delay();
    MPU_IIC_SCL = 1;
    I2C_Delay();
    while (MPU_READ_SDA) {
        ucErrTime++;
        if (ucErrTime > 250) {
            I2C_Stop();
            return 1;
        }
    }
    MPU_IIC_SCL = 0;
    return 0;
}

void I2C_Ack(void) {
    MPU_IIC_SCL = 0;
    MPU_SDA_OUT();
    MPU_IIC_SDA = 0;
    I2C_Delay();
    MPU_IIC_SCL = 1;
    I2C_Delay();
    MPU_IIC_SCL = 0;
}

void I2C_NoAck(void) {
    MPU_IIC_SCL = 0;
    MPU_SDA_OUT();
    MPU_IIC_SDA = 1;
    I2C_Delay();
    MPU_IIC_SCL = 1;
    I2C_Delay();
    MPU_IIC_SCL = 0;
}

void I2C_SendByte(uint8_t txd) {
    uint8_t t;
    MPU_SDA_OUT();
    MPU_IIC_SCL = 0;
    for (t = 0; t < 8; t++) {
        MPU_IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        MPU_IIC_SCL = 1;
        I2C_Delay();
        MPU_IIC_SCL = 0;
        I2C_Delay();
    }
}

uint8_t I2C_ReceiveByte(unsigned char ack) {
    unsigned char i, receive = 0;
    MPU_SDA_IN();
    for (i = 0; i < 8; i++) {
        MPU_IIC_SCL = 0;
        I2C_Delay();
        MPU_IIC_SCL = 1;
        receive <<= 1;
        if (MPU_READ_SDA)
            receive++;
        I2C_Delay();
    }
    if (!ack)
        I2C_NoAck();
    else
        I2C_Ack();
    return receive;
}

uint8_t I2C_Send(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len) {
    uint8_t i;
    I2C_Start();
    I2C_SendByte((addr << 1) | 0);
    if (I2C_WaitAck()) {
        I2C_Stop();
        return 1;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (i = 0; i < len; i++) {
        I2C_SendByte(buf[i]);
        if (I2C_WaitAck()) {
            I2C_Stop();
            return 1;
        }
    }
    I2C_Stop();
    return 0;
}

uint8_t I2C_Receive(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    I2C_Start();
    I2C_SendByte((addr << 1) | 0);
    if (I2C_WaitAck()) {
        I2C_Stop();
        return 1;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte((addr << 1) | 1);
    I2C_WaitAck();
    while (len) {
        if (len == 1)
            *buf = I2C_ReceiveByte(0);
        else
            *buf = I2C_ReceiveByte(1);
        len--;
        buf++;
    }
    I2C_Stop();
    return 0;
}