#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dmp.h"
#include "encoder.h"
#include "gpio.h"
#include "i2c.h"
#include "interrupt.h"
#include "motor.h"
#include "mpu.h"
#include "pid.h"
#include "pwm.h"
#include "rtc.h"
#include "serial.h"
#include "tim.h"
#include "usart.h"

#define MPU6050_DEVICE_ADDRESS ((uint8_t)0x68)
#define OFFESTADAPT_TIMES ((uint8_t)64)

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

I2C *GlobalI2C;
Serial *GlobalSerial;

MPU mpu;
Motor motor_left, motor_right;
Encoder encoder_left, encoder_right;
PID stand, speed, turn;

int16_t xacc_offset, yacc_offset, xgyro_offset, ygyro_offset, zgyro_offset;

int16_t PID_Stand(PID *pid, float pitch, int16_t ygyro);
int16_t PID_Speed(PID *pid, int16_t left, int16_t right);
int16_t PID_Turn(PID *pid, int16_t zgyro);

uint8_t WatchState = DISABLE;
void ReceiveHandler(Serial *serial);
void WatchHandler(Serial *serial);

int main() {
    RTC_Init();

    Serial serial = {
        .TX = "B10",
        .RX = "B11",
        .USARTx = USART3,
    };
    GlobalSerial = &serial;
    Serial_Init(&serial);
    Serial_SendString(
        &serial, "\r\n------------------------------------------------\r\n");
    INFO("Serial started\r\n");

    USART_Interrupt interrupt = {
        .USARTx = USART3,
        .USART_IT = USART_IT_RXNE,
        .NVIC_IRQChannel = USART3_IRQn,
        .NVIC_PriorityGroup = NVIC_PriorityGroup_2,
        .NVIC_IRQChannelPreemptionPriority = 2,
        .NVIC_IRQChannelSubPriority = 0,
    };
    INFO("starting USART interrupt\r\n");
    USART_Interrupt_Init(&interrupt);
    INFO("USART interrupt started\r\n");

    GPIO SCL = {
        .GPIOxPiny = "B8",
        .GPIO_Mode = GPIO_Mode_Out_PP,
    };
    GPIO SDA = {
        .GPIOxPiny = "B9",
        .GPIO_Mode = GPIO_Mode_Out_PP,
    };
    I2C i2c = {
        .SCL = &SCL,
        .SDA = &SDA,
        .frequency = 50000,
    };
    GlobalI2C = &i2c;
    mpu = (MPU){
        .i2c = &i2c,
        .DeviceAddress = MPU6050_DEVICE_ADDRESS,
    };
    INFO("starting MPU\r\n");
    MPU_Init(&mpu);
    INFO("MPU started\r\n");
    INFO("MPU adapting offset\r\n");
    MPU_AdaptOffset(&mpu, OFFESTADAPT_TIMES, &xacc_offset, &yacc_offset,
                    &xgyro_offset, &ygyro_offset, &zgyro_offset);
    INFO("MPU adapting offset succeeded\r\n");

    INFO("starting DMP\r\n");
    DMP_Init();
    INFO("DMP started\r\n");

    TIM tim_pwm = {
        RCC_APB2Periph_TIM1, TIM1, TIM_InternalClock, 100 - 1, 7200 - 1, CMD,
    };
    TIM_Init(&tim_pwm, NULL);
    Compare compare_left = {
        .TIMx = TIM1,
        .TIM_Pulse = 0,
        .TIM_OCInit = TIM_OC4Init,
        .TIM_SetCompare = TIM_SetCompare4,
    };
    GPIO gpio_pwm_left = {
        .GPIOxPiny = "A11",
        .GPIO_Mode = GPIO_Mode_AF_PP,
    };
    GPIO gpio_direction_left_1 = {
        .GPIOxPiny = "B12",
        .GPIO_Mode = GPIO_Mode_Out_PP,
    };
    GPIO gpio_direction_left_2 = {
        "B13",
        GPIO_Mode_Out_PP,
    };
    PWM pwm_left = {
        &tim_pwm,
        &compare_left,
        &gpio_pwm_left,
        DISABLE,
    };
    motor_left = (Motor){
        &pwm_left,
        &gpio_direction_left_1,
        &gpio_direction_left_2,
    };
    INFO("motor_left DMP\r\n");
    Motor_Init(&motor_left);
    INFO("motor_left started\r\n");

    Compare compare_right = {
        TIM1,
        0,
        TIM_OC1Init,
        TIM_SetCompare1,
    };
    GPIO gpio_pwm_right = {
        "A8",
        GPIO_Mode_AF_PP,
    };
    PWM pwm_right = {
        &tim_pwm,
        &compare_right,
        &gpio_pwm_right,
        DISABLE,
    };
    GPIO gpio_direction_right_1 = {
        "B14",
        GPIO_Mode_Out_PP,
    };
    GPIO gpio_direction_right_2 = {
        "B15",
        GPIO_Mode_Out_PP,
    };
    motor_right = (Motor){
        &pwm_right,
        &gpio_direction_right_1,
        &gpio_direction_right_2,
    };
    INFO("motor_right DMP\r\n");
    Motor_Init(&motor_right);
    INFO("motor_right started\r\n");

    GPIO gpio_encoder_left = {
        "A6 | A7",
        GPIO_Mode_IPU,
    };
    TIM tim_left = {
        RCC_APB1Periph_TIM3, TIM3, NULL, 1 - 1, 65536 - 1, UNCMD,
    };
    Capture capture_left1 = {
        TIM3, TIM_Channel_1,   TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI,
        0xF,  TIM_GetCapture1,
    };
    Capture capture_left2 = {
        TIM3, TIM_Channel_2,   TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI,
        0xF,  TIM_GetCapture2,
    };
    encoder_left = (Encoder){
        &gpio_encoder_left,    &tim_left,
        &capture_left1,        &capture_left2,
        TIM_ICPolarity_Rising, TIM_ICPolarity_Falling,
    };
    INFO("starting encoder_left\r\n");
    Encoder_Init(&encoder_left);
    INFO("encoder_left started\r\n");

    GPIO gpio_encoder_right = {
        .GPIOxPiny = "B6 | B7",
        .GPIO_Mode = GPIO_Mode_IPU,
    };
    TIM tim_right = {
        RCC_APB1Periph_TIM4, TIM4, NULL, 1 - 1, 65536 - 1, UNCMD,
    };
    Capture capture_right1 = {
        TIM4, TIM_Channel_1,   TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI,
        0xF,  TIM_GetCapture1,
    };
    Capture capture_right2 = {
        TIM4, TIM_Channel_2,   TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI,
        0xF,  TIM_GetCapture2,
    };
    encoder_right = (Encoder){
        &gpio_encoder_right,   &tim_right,
        &capture_right1,       &capture_right2,
        TIM_ICPolarity_Rising, TIM_ICPolarity_Rising,
    };
    INFO("starting encoder_right\r\n");
    Encoder_Init(&encoder_right);
    INFO("encoder_right started\r\n");

    stand = (PID){
        .KpState = ENABLE,
        .KiState = DISABLE,
        .KdState = ENABLE,
        .Kp = STAND_KP,
        .Ki = 0,
        .Kd = STAND_KD,
        .goal = ZERO,
    };
    PID_Init(&stand);
    speed = (PID){
        ENABLE, ENABLE, DISABLE, SPEED_KP, SPEED_KI, 0, 0,
    };
    PID_Init(&speed);
    turn = (PID){
        ENABLE, DISABLE, ENABLE, TURN_KP, 0, TURN_KD, 0,
    };
    PID_Init(&turn);

    TIM tim2 = {
        RCC_APB1Periph_TIM2, TIM2, TIM_InternalClock, 7200 - 1, 50 - 1, UNCMD,
    };
    INFO("starting TIM2\r\n");
    TIM_Init(&tim2, NULL);
    INFO("TIM2 started\r\n");

    TIM_Interrupt TIM_interrupt = {
        TIM2, TIM2_IRQn, NVIC_PriorityGroup_2, 0, 2,
    };
    INFO("starting TIM_Interrupt\r\n");
    TIM_Interrupt_Init(&TIM_interrupt);
    INFO("TIM_Interrupt started\r\n");

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

        stand_pulse = PID_Stand(&stand, pitch, ygyro - ygyro_offset);
        speed_pulse = PID_Speed(&speed, speed_left, speed_right);
        turn_pulse = PID_Turn(&turn, zgyro - zgyro_offset);

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
                } else if (strcmp(Body, "turnkp") == 0) {
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

// #include "stm32f10x.h"

// #include <string.h>

// #include "gpio.h"
// #include "i2c.h"
// #include "interrupt.h"
// #include "serial.h"
// #include "stm32f10x_gpio.h"

// I2C *GlobalI2C;
// Serial *GlobalSerial;

// void ReceiveHandler(Serial *serial);

// int main() {
//     RTC_Init();

//     GPIO TX = {
//         "B10",
//         GPIO_Mode_AF_PP,
//     };
//     GPIO RX = {
//         "B11",
//         GPIO_Mode_IPU,
//     };
//     USART usart = {
//         RCC_APB1Periph_USART3,
//         USART3,
//         USART_Mode_Tx | USART_Mode_Rx,
//     };
//     Serial serial = {
//         &TX,
//         &RX,
//         &usart,
//     };
//     GlobalSerial = &serial;
//     Serial_Init(&serial);
//     Serial_SendString(
//         &serial, "\r\n------------------------------------------------\r\n");
//     INFO("Serial started\r\n");

//     USART_Interrupt interrupt = {
//         USART3, USART_IT_RXNE, USART3_IRQn, NVIC_PriorityGroup_2, 2, 0,
//     };
//     INFO("starting USART interrupt\r\n");
//     USART_Interrupt_Init(&interrupt);
//     INFO("USART interrupt started\r\n");

//     INFO("start successfully\r\n");

//     GPIO gpio = {
//         .GPIOxPiny = "B15 | A0",
//         .GPIO_Mode = GPIO_Mode_Out_PP,
//     };
//     GPIO_Init_(&gpio);
//     GPIO_WriteBit(gpio.GPIOx, gpio.GPIO_Pin, Bit_SET);

//     for (;;) {
//     }
// }

// void USART3_IRQHandler(void) {
//     if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {

//         Serial_Parse(GlobalSerial);
//         ReceiveHandler(GlobalSerial);

//         USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//     }
// }

// void ReceiveHandler(Serial *serial) {
//     if (serial->RecieveFlag == SET) {
//         switch (serial->type) {
//         case Byte:
//             Serial_SendString(serial, "\r");
//             INFO("received Byte [%c]\r\n", serial->ByteData);

//             switch (serial->ByteData) {}
//             break;

//         case HexPack:
//             break;

//         case StringPack:
//             INFO("receive command [>%s]\r\n", serial->StringData);

//             char *Head = strtok(serial->StringData, " ");
//             if (strcmp(Head, "reset") == 0) {
//                 INFO("call reset succeeeded\r\n");
//                 NVIC_SystemReset();

//             } else {
//                 ERROR("unknow command\r\n");
//             }
//             break;

//         default:
//             break;
//         }

//         Serial_Clear(serial);
//     }
// }