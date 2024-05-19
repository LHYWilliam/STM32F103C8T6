#include "stm32f10x_gpio.h"

#include "delay.h"
#include "gpio.h"
#include "i2c.h"

void I2C_Init_(I2C *i2c) {
    GPIO_Init_(i2c->SCL);
    GPIO_Init_(i2c->SDA);

    GPIO_WriteBit(i2c->SCL->GPIOx, i2c->SCL->GPIO_Pin, (BitAction)1);
    GPIO_WriteBit(i2c->SDA->GPIOx, i2c->SDA->GPIO_Pin, (BitAction)1);
}

void I2C_Start(I2C *i2c) {
    SDA_OUT();
    I2C_SDA = 1;
    I2C_SCL = 1;
    I2C_Delay(i2c);
    I2C_SDA = 0;
    I2C_Delay(i2c);
    I2C_SCL = 0;
}

void I2C_Stop(I2C *i2c) {
    SDA_OUT();
    I2C_SCL = 0;
    I2C_SDA = 0;
    I2C_Delay(i2c);
    I2C_SCL = 1;
    I2C_SDA = 1;
    I2C_Delay(i2c);
}

uint8_t I2C_WaitAck(I2C *i2c) {
    uint8_t ucErrTime = 0;
    SDA_IN();
    I2C_SDA = 1;
    I2C_Delay(i2c);
    I2C_SCL = 1;
    I2C_Delay(i2c);
    while (I2C_READ_SDA) {
        ucErrTime++;
        if (ucErrTime > 250) {
            I2C_Stop(i2c);
            return 1;
        }
    }
    I2C_SCL = 0;

    return 0;
}

void I2C_Ack(I2C *i2c) {
    I2C_SCL = 0;
    SDA_OUT();
    I2C_SDA = 0;
    I2C_Delay(i2c);
    I2C_SCL = 1;
    I2C_Delay(i2c);
    I2C_SCL = 0;
}

void I2C_NoAck(I2C *i2c) {
    I2C_SCL = 0;
    SDA_OUT();
    I2C_SDA = 1;
    I2C_Delay(i2c);
    I2C_SCL = 1;
    I2C_Delay(i2c);
    I2C_SCL = 0;
}

void I2C_SendByte(I2C *i2c, uint8_t txd) {
    uint8_t t;
    SDA_OUT();
    I2C_SCL = 0;
    for (t = 0; t < 8; t++) {
        I2C_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        I2C_SCL = 1;
        I2C_Delay(i2c);
        I2C_SCL = 0;
        I2C_Delay(i2c);
    }
}

uint8_t I2C_ReceiveByte(I2C *i2c, unsigned char ack) {
    unsigned char i, receive = 0;
    SDA_IN();
    for (i = 0; i < 8; i++) {
        I2C_SCL = 0;
        I2C_Delay(i2c);
        I2C_SCL = 1;
        receive <<= 1;
        if (I2C_READ_SDA)
            receive++;
        I2C_Delay(i2c);
    }
    if (!ack)
        I2C_NoAck(i2c);
    else
        I2C_Ack(i2c);
    return receive;
}

void I2C_Delay(I2C *i2c) { Delay_us(2); }

uint8_t I2C_Send(I2C *i2c, uint8_t addr, uint8_t reg, const uint8_t *buf,
                 uint8_t len) {
    uint8_t i;
    I2C_Start(i2c);
    I2C_SendByte(i2c, (addr << 1) | 0);
    if (I2C_WaitAck(i2c)) {
        I2C_Stop(i2c);
        return 1;
    }
    I2C_SendByte(i2c, reg);
    I2C_WaitAck(i2c);
    for (i = 0; i < len; i++) {
        I2C_SendByte(i2c, buf[i]);
        if (I2C_WaitAck(i2c)) {
            I2C_Stop(i2c);
            return 1;
        }
    }
    I2C_Stop(i2c);
    return 0;
}

uint8_t I2C_Receive(I2C *i2c, uint8_t addr, uint8_t reg, uint8_t *buf,
                    uint8_t len) {
    I2C_Start(i2c);
    I2C_SendByte(i2c, (addr << 1) | 0);
    if (I2C_WaitAck(i2c)) {
        I2C_Stop(i2c);
        return 1;
    }
    I2C_SendByte(i2c, reg);
    I2C_WaitAck(i2c);
    I2C_Start(i2c);
    I2C_SendByte(i2c, (addr << 1) | 1);
    I2C_WaitAck(i2c);
    while (len) {
        if (len == 1)
            *buf = I2C_ReceiveByte(i2c, 0);
        else
            *buf = I2C_ReceiveByte(i2c, 1);
        len--;
        buf++;
    }
    I2C_Stop(i2c);
    return 0;
}