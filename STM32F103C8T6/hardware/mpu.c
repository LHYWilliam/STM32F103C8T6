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

void MPU_AdaptOffset(MPU *mpu, uint16_t times, int16_t *xacc_offset,
                     int16_t *yacc_offset, int16_t *xgyro_offset,
                     int16_t *ygyro_offset) {
    int16_t xacc, yacc, zacc, xgyro, ygyro, zgyro;
    int64_t xacc_sum = 0, yacc_sum = 0, xgyro_sum = 0, ygyro_sum = 0;

    for (uint16_t i = 0; i < times; i++) {
        MPU_GetData(mpu, &xacc, &yacc, &zacc, &xgyro, &ygyro, &zgyro);
        xacc_sum += xacc;
        yacc_sum += yacc;
        xgyro_sum += xgyro;
        ygyro_sum += ygyro;
    }

    *xacc_offset = (int16_t)(xacc_sum / times);
    *yacc_offset = (int16_t)(yacc_sum / times);
    *xgyro_offset = (int16_t)(xgyro_sum / times);
    *ygyro_offset = (int16_t)(ygyro_sum / times);
}

void MPU_GetData(MPU *mpu, int16_t *xacc, int16_t *yacc, int16_t *zacc,
                 int16_t *xgyro, int16_t *ygyro, int16_t *zgyro) {
    uint8_t acc[6], gyro[6];
    MPU_Receieve(mpu, MPU6050_ACCEL_XOUT_H, acc, 6);
    MPU_Receieve(mpu, MPU6050_GYRO_XOUT_H, gyro, 6);

    *xacc = (acc[0] << 8) | acc[1];
    *yacc = (acc[2] << 8) | acc[3];
    *zacc = (acc[4] << 8) | acc[5];
    *xgyro = (gyro[0] << 8) | gyro[1];
    *ygyro = (gyro[2] << 8) | gyro[3];
    *zgyro = (gyro[4] << 8) | gyro[5];
}

void MPU_Send(MPU *mpu, uint8_t RegisterAddress, const uint8_t *bytes,
              uint8_t length) {
    I2C_Send(mpu->i2c, mpu->DeviceAddress, RegisterAddress, bytes, length);
}
void MPU_Receieve(MPU *mpu, uint8_t RegisterAddress, uint8_t *bytes,
                  uint8_t length) {
    I2C_Receive(mpu->i2c, mpu->DeviceAddress, RegisterAddress, bytes, length);
}