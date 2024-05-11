#include "mpu.h"
#include "gpio.h"
#include "i2c.h"

void MPU_Init(MPU *mpu) {
    GPIO_Init_(mpu->SCL);
    GPIO_Init_(mpu->SDA);

    I2C_Init_(mpu->i2c);

    MPU_Cmd(mpu);
}

void MPU_Cmd(MPU *mpu) {
    const uint8_t PWR_MGMT_1 = 0x01;
    const uint8_t PWR_MGMT_2 = 0x00;

    const uint8_t SMPLRT_DIV = 0x09;

    const uint8_t CONFIG = 0x06;
    const uint8_t GYRO_CONFIG = 0x18;
    const uint8_t ACCEL_CONFIG = 0x18;

    MPU_Send(mpu, MPU6050_PWR_MGMT_1, &PWR_MGMT_1, 1);
    MPU_Send(mpu, MPU6050_PWR_MGMT_2, &PWR_MGMT_2, 1);

    MPU_Send(mpu, MPU6050_SMPLRT_DIV, &SMPLRT_DIV, 1);

    MPU_Send(mpu, MPU6050_CONFIG, &CONFIG, 1);
    MPU_Send(mpu, MPU6050_ACCEL_CONFIG, &ACCEL_CONFIG, 1);
    MPU_Send(mpu, MPU6050_GYRO_CONFIG, &GYRO_CONFIG, 1);
}

void MPU_GetData(MPU *mpu, int16_t *data) {
    uint8_t acc[6], gyro[6];
    MPU_Receieve(mpu, MPU6050_ACCEL_XOUT_H, acc, 6);
    MPU_Receieve(mpu, MPU6050_GYRO_XOUT_H, gyro, 6);

    for (uint8_t i = 0; i < 3; i++) {
        data[i] = (acc[2 * i] << 8) | acc[2 * i + 1];
    }
    for (uint8_t i = 0; i < 3; i++) {
        data[i + 3] = (gyro[2 * i] << 8) | gyro[2 * i + 1];
    }
}

void MPU_Send(MPU *mpu, uint8_t RegisterAddress, const uint8_t *bytes,
              uint8_t length) {
    I2C_Send(mpu->i2c, mpu->DeviceAddress, RegisterAddress, bytes, length);
}
void MPU_Receieve(MPU *mpu, uint8_t RegisterAddress, uint8_t *bytes,
                  uint8_t length) {
    I2C_Receive(mpu->i2c, mpu->DeviceAddress, RegisterAddress, bytes, length);
}