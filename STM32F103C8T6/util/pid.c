#include <math.h>
#include <stdint.h>

#include "pid.h"
#include "rtc.h"

float RC = 1 / (2 * 3.14 * 20);

void PID_Init(PID *pid) {
    pid->last_error = 0.;
    pid->integrator = 0.;
    pid->last_derivative = 0.;
    pid->last_time = 0;
}

int16_t PID_Caculate(PID *pid, float error) {
    uint32_t now = RTC_time_ms();
    float dt = (float)(now - pid->last_time) / 1000;

    if (pid->last_time == 0 || dt > 1) {
        pid->integrator = dt = 0;
        pid->last_derivative = 1e16;
    }
    pid->last_time = now;

    pid->output += error * pid->Kp;

    if (fabs(pid->Kd) > 0 && dt > 0) {
        float derivative;
        if (fabs(pid->last_derivative - 1e16) < 1e-6) {
            derivative = 0;
            pid->last_derivative = 0;
        } else {
            derivative = (error - pid->last_error) / dt;
        }

        derivative = pid->last_derivative +
                     (dt / (RC + dt)) * (derivative - pid->last_derivative);
        pid->last_error = error;
        pid->last_derivative = derivative;

        pid->output += pid->Kd * derivative;
    }

    if (fabs(pid->Ki) > 0 && dt > 0) {
        pid->integrator += error * pid->Ki * dt;
        LIMIT(pid->integrator, -pid->imax, pid->imax);

        pid->output += pid->integrator;
    }

    return (int32_t)pid->output;
}