#include "inv_mpu_dmp_motion_driver.h"

#include <math.h>
#include <stdlib.h>

#include "dmp.h"
#include "serial.h"

extern Serial *GlobalSerial;

static signed char gyro_orientation[9] = {-1, 0, 0, 0, -1, 0, 0, 0, 1};

void DMP_Init() {
    int8_t result;

    result = mpu_init(NULL);
    info("mpu_init %s\r\n", result ? "Failed" : "succeeded");

    result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    info("mpu_set_sensors %s\r\n", result ? "Failed" : "succeeded");
    result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    info("mpu_configure_fifo %s\r\n", result ? "Failed" : "succeeded");
    result = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    info("mpu_set_sample_rate %s\r\n", result ? "Failed" : "succeeded");

    result = dmp_load_motion_driver_firmware();
    info("dmp_load_motion_driver_firmware %s\r\n",
         result ? "Failed" : "succeeded");

    result =
        dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    info("dmp_set_orientation %s\r\n", result ? "Failed" : "succeeded");
    result = dmp_enable_feature(
        DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT |
        DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL);
    info("dmp_enable_feature %s\r\n", result ? "Failed" : "succeeded");

    result = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    info("dmp_set_fifo_rate %s\r\n", result ? "Failed" : "succeeded");

    info("runing self_test\r\n");
    run_self_test();
    info("self_test finished\r\n");

    result = mpu_set_dmp_state(1);
    info("mpu_set_dmp_state %s\r\n", result ? "Failed" : "succeeded");
}

void DMP_GetData(float *pitch, float *roll, float *yaw) {
    long quat[4];
    unsigned char more;
    short gyro[3], accel[3], sensors;
    unsigned long sensor_timestamp;
    float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
    q0 = quat[0] / q30;
    q1 = quat[1] / q30;
    q2 = quat[2] / q30;
    q3 = quat[3] / q30;

    *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
    *roll =
        atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
    *yaw =
        atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) *
        57.3;
}