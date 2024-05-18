#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include <stdlib.h>

#include "dmp.h"
#include "gpio.h"
#include "i2c.h"
#include "mpu.h"
#include "rtc.h"
#include "serial.h"
#include "usart.h"

#define MPU6050_DEVICE_ADDRESS ((uint8_t)0x68)

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
    logger("\r\nSerial started\r\n");

    I2C_Init_();
    MPU mpu = {
        MPU6050_DEVICE_ADDRESS,
    };
    logger("\r\nstarting MPU\r\n");
    MPU_Init(&mpu);
    logger("MPU started\r\n");

    logger("\r\nstarting RTC\r\n");
    RTC_Init();
    logger("RTC started\r\n");

    logger("\r\nstarting DMP\r\n");
    DMP_Init();
    logger("DMP started\r\n");

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
    }
}