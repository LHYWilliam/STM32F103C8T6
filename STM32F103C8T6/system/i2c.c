#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"

#include "i2c.h"

void I2C_Init_(I2C *i2c) {
    RCC_APB1PeriphClockCmd(i2c->RCC_APB1Periph, ENABLE);

    I2C_InitTypeDef I2C_InitStruct = {
        i2c->I2C_ClockSpeed, I2C_Mode_I2C,
        I2C_DutyCycle_2,     0x00,
        I2C_Ack_Enable,      I2C_AcknowledgedAddress_7bit,

    };
    I2C_Init(i2c->I2Cx, &I2C_InitStruct);

    I2C_Cmd(i2c->I2Cx, ENABLE);
}

void I2C_Send(I2C *i2c, uint8_t DeviceAddress, uint8_t RegisterAddress,
              const uint8_t *bytes, uint8_t length) {
    I2C_GenerateSTART(i2c->I2Cx, ENABLE);
    I2C_WaitEvent(i2c->I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(i2c->I2Cx, DeviceAddress, I2C_Direction_Transmitter);
    I2C_WaitEvent(i2c->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(i2c->I2Cx, RegisterAddress);
    I2C_WaitEvent(i2c->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING);

    for (uint8_t i = 0; i < length; i++) {
        I2C_SendData(i2c->I2Cx, bytes[i]);
        if (i < length - 1) {
            I2C_WaitEvent(i2c->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
        } else {
            I2C_WaitEvent(i2c->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        }
    }

    I2C_GenerateSTOP(i2c->I2Cx, ENABLE);
}
void I2C_Receive(I2C *i2c, uint8_t DeviceAddress, uint8_t RegisterAddress,
                 uint8_t *bytes, uint8_t length) {
    I2C_GenerateSTART(i2c->I2Cx, ENABLE);
    I2C_WaitEvent(i2c->I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(i2c->I2Cx, DeviceAddress, I2C_Direction_Transmitter);
    I2C_WaitEvent(i2c->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(i2c->I2Cx, RegisterAddress);
    I2C_WaitEvent(i2c->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING);

    I2C_GenerateSTART(i2c->I2Cx, ENABLE);
    I2C_WaitEvent(i2c->I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(i2c->I2Cx, DeviceAddress, I2C_Direction_Receiver);
    I2C_WaitEvent(i2c->I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

    for (uint8_t i = 0; i < length; i++) {
        I2C_WaitEvent(i2c->I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED);
        bytes[i] = I2C_ReceiveData(i2c->I2Cx);

        if (i == length - 2) {
            I2C_AcknowledgeConfig(i2c->I2Cx, DISABLE);
            I2C_GenerateSTOP(i2c->I2Cx, ENABLE);
        }
    }

    I2C_AcknowledgeConfig(i2c->I2Cx, ENABLE);
}

void I2C_WaitEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT) {
    uint16_t time_out = 10240;
    while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS && time_out--)
        ;
}