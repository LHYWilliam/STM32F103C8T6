#include <math.h>
#include <stdint.h>

#include "i2c.h"
#include "mpu.h"
#include "rtc.h"

void MPU_Init(MPU *mpu) { MPU_Cmd(mpu); }

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
                     int16_t *ygyro_offset, int16_t *zgyro_offset) {
    int16_t xacc, yacc, zacc, xgyro, ygyro, zgyro;
    int64_t xacc_sum = 0, yacc_sum = 0, xgyro_sum = 0, ygyro_sum = 0,
            zgyro_sum = 0;

    for (uint16_t i = 0; i < times; i++) {
        MPU_GetData(mpu, &xacc, &yacc, &zacc, &xgyro, &ygyro, &zgyro);
        xacc_sum += xacc;
        yacc_sum += yacc;
        xgyro_sum += xgyro;
        ygyro_sum += ygyro;
        zgyro_sum += zgyro;
    }

    *xacc_offset = (int16_t)(xacc_sum / times);
    *yacc_offset = (int16_t)(yacc_sum / times);
    *xgyro_offset = (int16_t)(xgyro_sum / times);
    *ygyro_offset = (int16_t)(ygyro_sum / times);
    *zgyro_offset = (int16_t)(zgyro_sum / times);
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

void MPU_Kalman(MPU *mpu, float *roll, float *pitch, int16_t xacc, int16_t yacc,
                int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro) {
    static uint32_t lasttime, now = 0.;
    static float k_roll = 0, k_pitch = 0;
    static const float rad2deg = 57.29578;
    static float e_P[2][2] = {{1, 0}, {0, 1}};
    static float k_k[2][2] = {{0, 0}, {0, 0}};

    now = RTC_time_ms();
    float dt = (now - lasttime) / 1000.0;
    lasttime = RTC_time_ms();

    float roll_v = xgyro +
                   ((sin(k_pitch) * sin(k_roll)) / cos(k_pitch)) * ygyro +
                   ((sin(k_pitch) * cos(k_roll)) / cos(k_pitch)) * zgyro;
    float pitch_v = cos(k_roll) * ygyro - sin(k_roll) * zgyro;
    float gyro_roll = k_roll + dt * roll_v;
    float gyro_pitch = k_pitch + dt * pitch_v;

    e_P[0][0] = e_P[0][0] + 0.0025;
    e_P[0][1] = e_P[0][1] + 0;
    e_P[1][0] = e_P[1][0] + 0;
    e_P[1][1] = e_P[1][1] + 0.0025;

    k_k[0][0] = e_P[0][0] / (e_P[0][0] + 0.3);
    k_k[0][1] = 0;
    k_k[1][0] = 0;
    k_k[1][1] = e_P[1][1] / (e_P[1][1] + 0.3);

    float acc_roll = atan(1.0 * yacc / zacc) * rad2deg;

    float acc_pitch =
        -1 * atan(xacc / sqrt(yacc * yacc + zacc * zacc)) * rad2deg;

    k_roll = gyro_roll + k_k[0][0] * (acc_roll - gyro_roll);
    k_pitch = gyro_pitch + k_k[1][1] * (acc_pitch - gyro_pitch);

    e_P[0][0] = (1 - k_k[0][0]) * e_P[0][0];
    e_P[0][1] = 0;
    e_P[1][0] = 0;
    e_P[1][1] = (1 - k_k[1][1]) * e_P[1][1];

    *roll = k_roll;
    *pitch = k_pitch;
}

void MPU_Send(MPU *mpu, uint8_t RegisterAddress, const uint8_t *bytes,
              uint8_t length) {
    I2C_Send(mpu->DeviceAddress, RegisterAddress, bytes, length);
}
void MPU_Receieve(MPU *mpu, uint8_t RegisterAddress, uint8_t *bytes,
                  uint8_t length) {
    I2C_Receive(mpu->DeviceAddress, RegisterAddress, bytes, length);
}