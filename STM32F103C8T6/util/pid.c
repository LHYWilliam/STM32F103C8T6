#include <stdint.h>

#include "pid.h"

void PID_Init(PID *pid) {
    pid->last = 0.;
    pid->sum = 0.;
}

void PID_SetGoal(PID *pid, float goal) { pid->goal = goal; }

int32_t PID_Caculate(PID *pid, float actual) {
    float error = pid->goal - actual;
    pid->sum += error;

    float result = pid->KpState ? pid->Kp * error : 0;
    result += pid->KiState ? pid->Ki * pid->sum : 0;
    result += pid->KdState ? pid->Kd * (error - pid->last) : 0;

    pid->last = error;

    return (int32_t)result;
}