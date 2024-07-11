#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dmp.h"
#include "encoder.h"
#include "i2c.h"
#include "motor.h"
#include "mpu.h"
#include "pid.h"
#include "rtc.h"
#include "serial.h"
#include "tim.h"

#define MPU6050_DEVICE_ADDRESS ((uint8_t)0x68)

#define ZERO -0.f

#define STAND_KP 700.f * 0.6
#define STAND_KD -3.0f * 0.6
#define SPEED_KP 250.f
#define SPEED_KI SPEED_KP / 200
#define TURN_KP 0.f
#define TURN_KD 0.f

// #define STAND_KP 800.f * 0.6
// #define STAND_KD -3.5f * 0.6
// #define SPEED_KP 300.f
// #define SPEED_KI SPEED_KP / 200
// #define TURN_KP 0.f
// #define TURN_KD 0.f

// #define STAND_KP 640.f * 0.6
// #define STAND_KD -3.4f * 0.6 - 0.2
// #define SPEED_KP 375.f
// #define SPEED_KI SPEED_KP / 200
// #define TURN_KP 0.f
// #define TURN_KD 0.f

Serial serial = {
    .TX = "B10",
    .RX = "B11",
    .USARTx = USART3,
};

I2C i2c = {
    .SCL = "B8",
    .SDA = "B9",
};

MPU mpu = {
    .DeviceAddress = MPU6050_DEVICE_ADDRESS,
};

Motor motor_left = {
    .pwm = "A11",
    .direction1 = "B12",
    .direction2 = "B13",
    .TIMx = TIM1,
    .channel = 4,
    .TIM_Init_Mode = ENABLE,
};

Motor motor_right = {
    .pwm = "A8",
    .direction1 = "B14",
    .direction2 = "B15",
    .TIMx = TIM1,
    .channel = 1,
    .TIM_Init_Mode = DISABLE,
};

Encoder encoder_left = {
    .gpio = "A6 | A7",
    .TIMx = TIM3,
    .TIM_IC1Polarity = TIM_ICPolarity_Rising,
    .TIM_IC2Polarity = TIM_ICPolarity_Falling,
};

Encoder encoder_right = {
    .gpio = "B6 | B7",
    .TIMx = TIM4,
    .TIM_IC1Polarity = TIM_ICPolarity_Rising,
    .TIM_IC2Polarity = TIM_ICPolarity_Rising,
};

PID stand = {
    .Kp = STAND_KP,
    .Kd = STAND_KD,
    .goal = ZERO,
};

PID speed = {
    .Kp = SPEED_KP,
    .Ki = SPEED_KI,
    .goal = 0,
};

PID turn = {
    .Kp = TURN_KP,
    .Kd = TURN_KD,
    .goal = 0,
};

Timer timer = {
    .TIMx = TIM2,
    .ms = 5,
};

I2C *GlobalI2C = &i2c;
Serial *GlobalSerial = &serial;

uint8_t WatchState = DISABLE;

int16_t PID_Stand(PID *pid, float pitch, int16_t ygyro);
int16_t PID_Speed(PID *pid, int16_t left, int16_t right);
int16_t PID_Turn(PID *pid, int16_t zgyro);

void ReceiveHandler(Serial *serial);
void WatchHandler(Serial *serial);

int main() {
    RTC_Init();

    Serial_Init(&serial);
    Serial_SendString(
        &serial, "\r\n------------------------------------------------\r\n");
    INFO("Serial started\r\n");

    INFO("starting I2C\r\n");
    I2C_Init_(&i2c);
    INFO("I2C started\r\n");

    INFO("starting MPU\r\n");
    MPU_Init(&mpu);
    INFO("MPU started\r\n");

    INFO("starting DMP\r\n");
    DMP_Init();
    INFO("DMP started\r\n");

    INFO("motor_left DMP\r\n");
    Motor_Init(&motor_left);
    INFO("motor_left started\r\n");

    INFO("motor_right DMP\r\n");
    Motor_Init(&motor_right);
    INFO("motor_right started\r\n");

    INFO("starting encoder_left\r\n");
    Encoder_Init(&encoder_left);
    INFO("encoder_left started\r\n");

    INFO("starting encoder_right\r\n");
    Encoder_Init(&encoder_right);
    INFO("encoder_right started\r\n");

    INFO("starting PID\r\n");
    PID_Init(&stand);
    PID_Init(&speed);
    PID_Init(&turn);
    INFO("PID started\r\n");

    INFO("starting timer\r\n");
    Timer_Init(&timer);
    INFO("timer started\r\n");

    INFO("started successfully\r\n");

    for (;;) {
        if (WatchState) {
            WatchHandler(GlobalSerial);
        }
    }
}

void TIM2_IRQHandler(void) {
    static float pitch, roll, yaw;
    static int16_t xacc, yacc, zacc, xgyro, ygyro, zgyro;

    static int16_t speed_left, speed_right;
    static int16_t left_pulse, right_pulse;
    static int16_t stand_pulse, speed_pulse, turn_pulse;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {

        DMP_GetData(&pitch, &roll, &yaw);
        MPU_GetData(&mpu, &xacc, &yacc, &zacc, &xgyro, &ygyro, &zgyro);

        speed_left = Encoder_Get(&encoder_left);
        speed_right = Encoder_Get(&encoder_right);

        stand_pulse = PID_Stand(&stand, pitch, ygyro);
        speed_pulse = PID_Speed(&speed, speed_left, speed_right);
        turn_pulse = PID_Turn(&turn, zgyro);

        left_pulse = stand_pulse + speed_pulse + turn_pulse;
        right_pulse = stand_pulse + speed_pulse - turn_pulse;
        LIMIT(left_pulse, -7200, 7200);
        LIMIT(right_pulse, -7200, 7200);

        Motor_Set(&motor_left, left_pulse - 1);
        Motor_Set(&motor_right, right_pulse - 1);

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

void USART3_IRQHandler(void) {
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {

        Serial_Parse(GlobalSerial);
        ReceiveHandler(GlobalSerial);

        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}

int16_t PID_Stand(PID *pid, float pitch, int16_t ygyro) {
    static float error;
    error = pitch - pid->goal;

    return pid->Kp * error + pid->Kd * ygyro;
}

int16_t PID_Speed(PID *pid, int16_t left, int16_t right) {
    static float error;
    error = left + right - pid->goal;
    error = pid->last * 0.7 + error * 0.3;

    pid->sum += error;
    pid->last = error;

    LIMIT(pid->sum, -7200, 7200);

    return pid->Kp * error + pid->Ki * pid->sum;
}

int16_t PID_Turn(PID *pid, int16_t zgyro) {
    return pid->goal == 0. ? pid->Kd * zgyro : pid->Kp * pid->goal;
}

void ReceiveHandler(Serial *serial) {
    if (serial->RecieveFlag == SET) {
        switch (serial->type) {
        case Byte:
            Serial_SendString(serial, "\r");
            INFO("received Byte [%c]\r\n", serial->ByteData);

            switch (serial->ByteData) {
            case 0x0D:
                WatchState = DISABLE;
                break;

            case 0x57:
                speed.goal = speed.goal < 0 ? 0 : speed.goal + 10;
                break;

            case 0x53:
                speed.goal = speed.goal > 0 ? 0 : speed.goal - 10;
                break;

            case 0x41:
                turn.goal = turn.goal > 0 ? 0 : turn.goal - 10;
                break;

            case 0x44:
                turn.goal = turn.goal < 0 ? 0 : turn.goal + 10;
                break;
            }
            break;

        case HexPack:
            break;

        case StringPack:
            INFO("receive command [>%s]\r\n", serial->StringData);

            char *Head = strtok(serial->StringData, " ");
            if (strcmp(Head, "set") == 0) {
                char *Body = strtok(NULL, " ");

                if (strcmp(Body, "zero") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%f", &stand.goal);
                    INFO("successfully set %s to %f\r\n", Body, stand.goal);
                } else if (strcmp(Body, "speed") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%f", &speed.goal);
                    INFO("successfully set %s to %f\r\n", Body, speed.goal);
                } else if (strcmp(Body, "turn") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%f", &turn.goal);
                    INFO("successfully set %s to %f\r\n", Body, turn.goal);
                } else if (strcmp(Body, "standkp") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%f", &stand.Kp);
                    INFO("successfully set %s to %f\r\n", Body, stand.Kp);
                } else if (strcmp(Body, "standkd") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%f", &stand.Kd);
                    INFO("successfully set %s to %f\r\n", Body, stand.Kd);
                } else if (strcmp(Body, "speedkp") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%f", &speed.Kp);
                    speed.Ki = speed.Kp / 200;
                    INFO("successfully set %s to %f\r\n", Body, speed.Kp);
                } else if (strcmp(Body, "turnkp") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%f", &turn.Kp);
                    INFO("successfully set %s to %f\r\n", Body, turn.Kp);
                } else if (strcmp(Body, "turnkd") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%f", &turn.Kd);
                    INFO("successfully set %s to %f\r\n", Body, turn.Kd);
                } else {
                    ERROR("unknow command\r\n");
                }

            } else if (strcmp(Head, "call") == 0) {

            } else if (strcmp(Head, "show") == 0) {
                char *Body = strtok(NULL, " ");

                if (strcmp(Body, "zero") == 0) {
                    INFO("%s: %f\r\n", Body, stand.goal);
                } else if (strcmp(Body, "standkp") == 0) {
                    INFO("%s: %f\r\n", Body, stand.Kp);
                } else if (strcmp(Body, "standkd") == 0) {
                    INFO("%s: %f\r\n", Body, stand.Kd);
                } else if (strcmp(Body, "speedkp") == 0) {
                    INFO("%s: %f\r\n", Body, speed.Kp);
                } else if (strcmp(Body, "turnkp") == 0) {
                    INFO("%s: %f\r\n", Body, turn.Kp);
                } else if (strcmp(Body, "turnkd") == 0) {
                    INFO("%s: %f\r\n", Body, turn.Kd);
                } else {
                    ERROR("unknow command\r\n");
                }

            } else if (strcmp(Head, "watch") == 0) {
                WatchState = ENABLE;

            } else if (strcmp(Head, "reset") == 0) {
                INFO("call reset succeeeded\r\n");
                NVIC_SystemReset();

            } else {
                ERROR("unknow command\r\n");
            }
            break;

        default:
            break;
        }

        Serial_Clear(serial);
    }
}

void WatchHandler(Serial *serial) {}