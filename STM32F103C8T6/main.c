#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "gpio.h"
#include "i2c.h"
#include "mpu.h"
#include "oled.h"
#include <stdint.h>

int main() {
    OLED_Init();

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

    int16_t data[6];
    for (;;) {
        MPU_GetData(&mpu, data);

        OLED_ShowSignedNum(2, 1, data[0], 5);
        OLED_ShowSignedNum(3, 1, data[1], 5);
        OLED_ShowSignedNum(4, 1, data[2], 5);
        OLED_ShowSignedNum(2, 8, data[3], 5);
        OLED_ShowSignedNum(3, 8, data[4], 5);
        OLED_ShowSignedNum(4, 8, data[5], 5);
    }
}