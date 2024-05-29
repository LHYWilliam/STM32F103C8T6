#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    uint8_t KpState;
    uint8_t KiState;
    uint8_t KdState;

    float Kp;
    float Ki;
    float Kd;

    float goal;

    float last;
    float sum;
} PID;

void PID_Init(PID *pid);

void PID_SetGoal(PID *pid, float goal);
int32_t PID_Caculate(PID *pid, float actual);

#endif