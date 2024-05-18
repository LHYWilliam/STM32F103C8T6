#include "stm32f10x.h"

#include "delay.h"
#include "softi2c.h"

#include "softi2c.h"

// MPU IIC 延时函数
void MPU_IIC_Delay(void) { Delay_us(2); }

// 初始化IIC
void MPU_IIC_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,
                           ENABLE); // 先使能外设IO PORTB时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // IO口速度为50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure); // 根据设定参数初始化GPIO

    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11); // PB10,PB11 输出高
}
// 产生IIC起始信号
void MPU_IIC_Start(void) {
    MPU_SDA_OUT(); // sda线输出
    MPU_IIC_SDA = 1;
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay();
    MPU_IIC_SDA = 0; // START:when CLK is high,DATA change form high to low
    MPU_IIC_Delay();
    MPU_IIC_SCL = 0; // 钳住I2C总线，准备发送或接收数据
}
// 产生IIC停止信号
void MPU_IIC_Stop(void) {
    MPU_SDA_OUT(); // sda线输出
    MPU_IIC_SCL = 0;
    MPU_IIC_SDA = 0; // STOP:when CLK is high DATA change form low to high
    MPU_IIC_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_SDA = 1; // 发送I2C总线结束信号
    MPU_IIC_Delay();
}
// 等待应答信号到来
// 返回值：1，接收应答失败
//         0，接收应答成功
uint8_t MPU_IIC_Wait_Ack(void) {
    uint8_t ucErrTime = 0;
    MPU_SDA_IN(); // SDA设置为输入
    MPU_IIC_SDA = 1;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay();
    while (MPU_READ_SDA) {
        ucErrTime++;
        if (ucErrTime > 250) {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_SCL = 0; // 时钟输出0
    return 0;
}
// 产生ACK应答
void MPU_IIC_Ack(void) {
    MPU_IIC_SCL = 0;
    MPU_SDA_OUT();
    MPU_IIC_SDA = 0;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 0;
}
// 不产生ACK应答
void MPU_IIC_NAck(void) {
    MPU_IIC_SCL = 0;
    MPU_SDA_OUT();
    MPU_IIC_SDA = 1;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 0;
}
// IIC发送一个字节
// 返回从机有无应答
// 1，有应答
// 0，无应答
void MPU_IIC_Send_Byte(uint8_t txd) {
    uint8_t t;
    MPU_SDA_OUT();
    MPU_IIC_SCL = 0; // 拉低时钟开始数据传输
    for (t = 0; t < 8; t++) {
        MPU_IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        MPU_IIC_SCL = 1;
        MPU_IIC_Delay();
        MPU_IIC_SCL = 0;
        MPU_IIC_Delay();
    }
}
// 读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t MPU_IIC_Read_Byte(unsigned char ack) {
    unsigned char i, receive = 0;
    MPU_SDA_IN(); // SDA设置为输入
    for (i = 0; i < 8; i++) {
        MPU_IIC_SCL = 0;
        MPU_IIC_Delay();
        MPU_IIC_SCL = 1;
        receive <<= 1;
        if (MPU_READ_SDA)
            receive++;
        MPU_IIC_Delay();
    }
    if (!ack)
        MPU_IIC_NAck(); // 发送nACK
    else
        MPU_IIC_Ack(); // 发送ACK
    return receive;
}

uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf) {
    uint8_t i;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack())             // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // 写寄存器地址
    MPU_IIC_Wait_Ack();     // 等待应答
    for (i = 0; i < len; i++) {
        MPU_IIC_Send_Byte(buf[i]); // 发送数据
        if (MPU_IIC_Wait_Ack())    // 等待ACK
        {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_Stop();
    return 0;
}
// IIC连续读
// addr:器件地址
// reg:要读取的寄存器地址
// len:要读取的长度
// buf:读取到的数据存储区
// 返回值:0,正常
//     其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf) {
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack())             // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // 写寄存器地址
    MPU_IIC_Wait_Ack();     // 等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 1); // 发送器件地址+读命令
    MPU_IIC_Wait_Ack();                 // 等待应答
    while (len) {
        if (len == 1)
            *buf = MPU_IIC_Read_Byte(0); // 读数据,发送nACK
        else
            *buf = MPU_IIC_Read_Byte(1); // 读数据,发送ACK
        len--;
        buf++;
    }
    MPU_IIC_Stop(); // 产生一个停止条件
    return 0;
}
// IIC写一个字节
// reg:寄存器地址
// data:数据
// 返回值:0,正常
//     其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data) {
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack())                 // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);  // 写寄存器地址
    MPU_IIC_Wait_Ack();      // 等待应答
    MPU_IIC_Send_Byte(data); // 发送数据
    if (MPU_IIC_Wait_Ack())  // 等待ACK
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Stop();
    return 0;
}
// IIC读一个字节
// reg:寄存器地址
// 返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg) {
    uint8_t res;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0); // 发送器件地址+写命令
    MPU_IIC_Wait_Ack();                     // 等待应答
    MPU_IIC_Send_Byte(reg);                 // 写寄存器地址
    MPU_IIC_Wait_Ack();                     // 等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 1); // 发送器件地址+读命令
    MPU_IIC_Wait_Ack();                     // 等待应答
    res = MPU_IIC_Read_Byte(0);             // 读取数据,发送nACK
    MPU_IIC_Stop();                         // 产生一个停止条件
    return res;
}
