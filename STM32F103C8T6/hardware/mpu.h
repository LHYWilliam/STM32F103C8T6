#ifndef __MPU_H
#define __MPU_H

#include "gpio.h"
#include "i2c.h"

#define MPU6050_DEVICE_ADDRESS ((uint8_t)(0x68 << 1))

#define MPU6050_PWR_MGMT_1 ((uint8_t)0x6B)
#define MPU6050_PWR_MGMT_2 ((uint8_t)0x6C)
#define MPU6050_WHO_AM_I ((uint8_t)0x75)

#define MPU6050_SMPLRT_DIV ((uint8_t)0x19)
#define MPU6050_CONFIG ((uint8_t)0x1A)
#define MPU6050_GYRO_CONFIG ((uint8_t)0x1B)
#define MPU6050_ACCEL_CONFIG ((uint8_t)0x1C)

#define MPU6050_ACCEL_XOUT_H ((uint8_t)0x3B)
#define MPU6050_ACCEL_XOUT_L ((uint8_t)0x3C)

#define MPU6050_ACCEL_YOUT_H ((uint8_t)0x3D)
#define MPU6050_ACCEL_YOUT_L ((uint8_t)0x3E)

#define MPU6050_ACCEL_ZOUT_H ((uint8_t)0x3F)
#define MPU6050_ACCEL_ZOUT_L ((uint8_t)0x40)

#define MPU6050_TEMP_OUT_H ((uint8_t)0x41)
#define MPU6050_TEMP_OUT_L ((uint8_t)0x42)

#define MPU6050_GYRO_XOUT_H ((uint8_t)0x43)
#define MPU6050_GYRO_XOUT_L ((uint8_t)0x44)

#define MPU6050_GYRO_YOUT_H ((uint8_t)0x45)
#define MPU6050_GYRO_YOUT_L ((uint8_t)0x46)

#define MPU6050_GYRO_ZOUT_H ((uint8_t)0x47)
#define MPU6050_GYRO_ZOUT_L ((uint8_t)0x48)

typedef struct {
    GPIO *SCL;
    GPIO *SDA;

    I2C *i2c;

    uint8_t DeviceAddress;
} MPU;

void MPU_Init(MPU *mpu);
void MPU_Cmd(MPU *mpu);
void MPU_AdaptOffset(MPU *mpu, uint16_t times, int16_t *xacc_offset,
                     int16_t *yacc_offset, int16_t *xgyro_offset,
                     int16_t *ygyro_offset, int16_t *zgyro_offset);
void MPU_GetData(MPU *mpu, int16_t *xacc, int16_t *yacc, int16_t *zacc,
                 int16_t *xgyro, int16_t *ygyro, int16_t *zgyro);
void MPU_Kalman(MPU *mpu, float *roll, float *pitch, int16_t xacc, int16_t yacc,
                int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro);
void MPU_Send(MPU *mpu, uint8_t RegisterAddress, const uint8_t *bytes,
              uint8_t length);
void MPU_Receieve(MPU *mpu, uint8_t RegisterAddress, uint8_t *bytes,
                  uint8_t length);

#endif