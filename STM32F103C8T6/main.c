#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include <stdlib.h>

#include "delay.h"
#include "dmp.h"
#include "gpio.h"
#include "i2c.h"
#include "mpu.h"
#include "rtc.h"
#include "serial.h"
#include "softi2c.h"
#include "usart.h"

I2C *GlobalI2C;
Serial *GlobalSerial;

int main() {
    GPIO gpio_TX = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_9,
        GPIO_Mode_AF_PP,
    };
    USART usart = {
        RCC_APB2Periph_USART1,
        USART1,
        USART_Mode_Tx,
    };
    Serial serial = {
        &gpio_TX,
        NULL,
        &usart,
    };
    GlobalSerial = &serial;
    Serial_Init(&serial);
    Serial_SendString(&serial, "\r\nSerial started\r\n");

    // GPIO SCL = {
    //     RCC_APB2Periph_GPIOB,
    //     GPIOB,
    //     GPIO_Pin_10,
    //     GPIO_Mode_AF_OD,
    // };
    // GPIO SDA = {
    //     RCC_APB2Periph_GPIOB,
    //     GPIOB,
    //     GPIO_Pin_11,
    //     GPIO_Mode_AF_OD,
    // };
    // I2C i2c = {
    //     RCC_APB1Periph_I2C2,
    //     I2C2,
    //     10000,
    // };
    // GlobalI2C = &i2c;
    // MPU mpu = {
    //     &SCL,
    //     &SDA,
    //     &i2c,
    //     MPU6050_DEVICE_ADDRESS,
    // };
    MPU_IIC_Init();
    // Serial_SendString(&serial, "\r\nstarting MPU\r\n");
    // MPU_Init(&mpu);
    // Serial_SendString(&serial, "MPU started\r\n");

    Serial_SendString(&serial, "\r\nstarting RTC\r\n");
    RTC_Init();
    Serial_SendString(&serial, "RTC started\r\n");

    Serial_SendString(&serial, "\r\nstarting DMP\r\n");
    DMP_Init();
    Serial_SendString(&serial, "DMP started\r\n");

    float roll = 0, pitch = 0, yaw = 0;
    int16_t xacc, yacc, zacc, xgyro, ygyro, zgyro;
    int16_t xacc_offset, yacc_offset, xgyro_offset, ygyro_offset, zgyro_offset;

    // Serial_SendString(&serial, "\r\nAdapting offset\r\n");
    // MPU_AdaptOffset(&mpu, 256, &xacc_offset, &yacc_offset, &xgyro_offset,
    //                 &ygyro_offset, &zgyro_offset);
    // Serial_SendString(&serial, "offset adapted\r\n");

    for (;;) {
        // Delay_ms(100);
        // MPU_GetData(&mpu, &xacc, &yacc, &zacc, &xgyro, &ygyro, &zgyro);

        // Serial_SendString(&serial, "%+6d %+6d %+6d %+6d %+6d %+6d\r",
        //                   xacc - xacc_offset, yacc - yacc_offset, zacc,
        //                   xgyro - xgyro_offset, ygyro - ygyro_offset,
        //                   zgyro - zgyro_offset);
        DMP_GetData(&pitch, &roll, &yaw);
        Serial_SendString(&serial, "pitch:%+8.2f roll:%+8.2f yaw:%+8.2f\r",
                          pitch, roll, yaw);

        // MPU_Kalman(&mpu, &roll, &pitch, xacc, yacc, zacc, xgyro, ygyro,
        // zgyro);
    }
}