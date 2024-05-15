#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include <stdlib.h>

#include "delay.h"
#include "gpio.h"
#include "i2c.h"
#include "mpu.h"
#include "rtc.h"
#include "serial.h"
#include "usart.h"

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
    Serial_Init(&serial);

    GPIO SCL = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_10,
        GPIO_Mode_AF_OD,
    };
    GPIO SDA = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_11,
        GPIO_Mode_AF_OD,
    };
    I2C i2c = {
        RCC_APB1Periph_I2C2,
        I2C2,
        50000,
    };
    MPU mpu = {
        &SCL,
        &SDA,
        &i2c,
        MPU6050_DEVICE_ADDRESS,
    };
    MPU_Init(&mpu);

    int16_t xacc, yacc, zacc, xgyro, ygyro, zgyro;
    int16_t xacc_offset, yacc_offset, xgyro_offset, ygyro_offset, zgyro_offset;

    MPU_AdaptOffset(&mpu, 256, &xacc_offset, &yacc_offset, &xgyro_offset,
                    &ygyro_offset, &zgyro_offset);

    RTC_Init();
    for (;;) {
        MPU_GetData(&mpu, &xacc, &yacc, &zacc, &xgyro, &ygyro, &zgyro);

        Serial_SendString(&serial, "time: %4d    ", RTC_time());
        Serial_SendString(&serial, "%+6d %+6d %+6d %+6d %+6d %+6d\r\n",
                          xacc - xacc_offset, yacc - yacc_offset, zacc,
                          xgyro - xgyro_offset, ygyro - ygyro_offset,
                          zgyro - zgyro_offset);
    }
}