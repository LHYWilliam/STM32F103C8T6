#include "inv_mpu_dmp_motion_driver.h"

#include <math.h>
#include <stdlib.h>

#include "dmp.h"
#include "serial.h"

// #define DMP_SELFTEST

extern Serial *GlobalSerial;

static signed char gyro_orientation[9] = {-1, 0, 0, 0, -1, 0, 0, 0, 1};

void DMP_Init() {
    int8_t result;

    if (result = mpu_init(NULL), result == 0) {
        info("mpu_init succeeded\r\n");
    } else {
        error("mpu_init failed %d\r\n", result);
    }

    if (result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL), result == 0) {
        info("mpu_set_sensors succeeded\r\n");
    } else {
        error("mpu_set_sensors failed %d\r\n", result);
    }
    if (result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL),
        result == 0) {
        info("mpu_configure_fifo succeeded\r\n");
    } else {
        error("mpu_configure_fifo failed %d\r\n", result);
    }
    if (result = mpu_set_sample_rate(DEFAULT_MPU_HZ), result == 0) {
        info("mpu_set_sample_rate succeeded\r\n");
    } else {
        error("mpu_set_sample_rate failed %d\r\n", result);
    }

    if (result = dmp_load_motion_driver_firmware(), result == 0) {
        info("dmp_load_motion_driver_firmware succeeded\r\n");
    } else {
        error("dmp_load_motion_driver_firmware failed %d\r\n", result);
    }

    if (result = dmp_set_orientation(
            inv_orientation_matrix_to_scalar(gyro_orientation)),
        result == 0) {
        info("dmp_set_orientation succeeded\r\n");
    } else {
        error("dmp_set_orientation failed %d\r\n", result);
    }
    if (result = dmp_enable_feature(
            DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
            DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL),
        result == 0) {
        info("dmp_enable_feature succeeded\r\n");
    } else {
        error("dmp_enable_feature failed %d\r\n", result);
    }

    if (result = dmp_set_fifo_rate(DEFAULT_MPU_HZ), result == 0) {
        info("dmp_set_fifo_rate succeeded\r\n");
    } else {
        error("dmp_set_fifo_rate failed %d\r\n", result);
    }

#ifdef DMP_SELFTEST
    info("runing self_test\r\n");
    run_self_test();
    info("self_test finished\r\n");
#endif

    if (result = mpu_set_dmp_state(1), result == 0) {
        info("mpu_set_dmp_state succeeded\r\n");
    } else {
        error("mpu_set_dmp_state failed %d\r\n", result);
    }
}

void DMP_GetData(float *pitch, float *roll, float *yaw) {
    long quat[4];
    unsigned char more;
    short gyro[3], accel[3], sensors;
    unsigned long sensor_timestamp;
    float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

    while (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
        ;
    if (sensors & INV_WXYZ_QUAT) {
        q0 = quat[0] / q30;
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;

        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
        *roll =
            atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) *
            57.3;
        *yaw = atan2(2 * (q1 * q2 + q0 * q3),
                     q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) *
               57.3;
    }
}