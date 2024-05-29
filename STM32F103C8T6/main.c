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

#define ZERO 0.f
#define STAND_KP -300.0f
#define STAND_KD 0.1f
#define OFFESTADAPT_TIMES ((uint8_t)64)
#define MPU6050_DEVICE_ADDRESS ((uint8_t)0x68)

I2C *GlobalI2C;
MPU *GlobalMPU;
PID *GlobalStand;
Serial *GlobalSerial;

uint8_t WatchState = DISABLE;

int16_t PID_StandCaculate(PID *pid, float pitch, int16_t ygyro);

void ReceiveHandler(Serial *serial);
void WatchHandler(Serial *serial);

Motor *left, *right;

float pitch, roll, yaw;
int16_t xacc, yacc, zacc, xgyro, ygyro, zgyro;
int16_t xacc_offset, yacc_offset, xgyro_offset, ygyro_offset, zgyro_offset;

int16_t speed_left = 0, speed_right = 0;
int16_t pulse_left = 0, pulse_right = 0;

int main() {
    RTC_Init();

    GPIO TX = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_10,
        GPIO_Mode_AF_PP,
    };
    GPIO RX = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_11,
        GPIO_Mode_IPU,
    };
    USART usart = {
        RCC_APB1Periph_USART3,
        USART3,
        USART_Mode_Tx | USART_Mode_Rx,
    };
    Serial serial = {
        &TX,
        &RX,
        &usart,
    };
    GlobalSerial = &serial;
    Serial_Init(&serial);
    Serial_SendString(
        &serial, "\r\n------------------------------------------------\r\n");
    info("Serial started\r\n");

    USART_Interrupt interrupt = {
        USART3, USART_IT_RXNE, USART3_IRQn, NVIC_PriorityGroup_2, 2, 0,
    };
    info("starting USART interrupt\r\n");
    USART_Interrupt_Init(&interrupt);
    info("USART interrupt started\r\n");

    GPIO SCL = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_8,
        GPIO_Mode_Out_PP,
    };
    GPIO SDA = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_9,
        GPIO_Mode_Out_PP,
    };
    I2C i2c = {
        &SCL,
        &SDA,
        50000,
    };
    GlobalI2C = &i2c;
    MPU mpu = {
        &i2c,
        MPU6050_DEVICE_ADDRESS,
    };
    GlobalMPU = &mpu;
    info("starting MPU\r\n");
    MPU_Init(&mpu);
    info("MPU started\r\n");
    info("MPU adapting offset\r\n");
    MPU_AdaptOffset(&mpu, OFFESTADAPT_TIMES, &xacc_offset, &yacc_offset,
                    &xgyro_offset, &ygyro_offset, &zgyro_offset);
    info("MPU adapting offset succeeded\r\n");

    info("starting DMP\r\n");
    DMP_Init();
    info("DMP started\r\n");

    TIM tim_pwm = {
        RCC_APB2Periph_TIM1, TIM1, TIM_InternalClock, 100 - 1, 7200 - 1, CMD,
    };
    TIM_Init(&tim_pwm, NULL);
    Compare compare_left = {
        TIM1,
        0,
        TIM_OC4Init,
        TIM_SetCompare4,
    };
    GPIO gpio_pwm_left = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_11,
        GPIO_Mode_AF_PP,
    };
    GPIO gpio_direction_left_1 = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_12,
        GPIO_Mode_Out_PP,
    };
    GPIO gpio_direction_left_2 = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_13,
        GPIO_Mode_Out_PP,
    };
    PWM pwm_left = {
        &tim_pwm,
        &compare_left,
        &gpio_pwm_left,
        DISABLE,
    };
    Motor motor_left = {
        &pwm_left,
        &gpio_direction_left_1,
        &gpio_direction_left_2,
    };
    left = &motor_left;
    info("motor_left DMP\r\n");
    Motor_Init(&motor_left);
    info("motor_left started\r\n");

    Compare compare_right = {
        TIM1,
        0,
        TIM_OC1Init,
        TIM_SetCompare1,
    };
    GPIO gpio_pwm_right = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_8,
        GPIO_Mode_AF_PP,
    };
    PWM pwm_right = {
        &tim_pwm,
        &compare_right,
        &gpio_pwm_right,
        DISABLE,
    };
    GPIO gpio_direction_right_1 = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_14,
        GPIO_Mode_Out_PP,
    };
    GPIO gpio_direction_right_2 = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_15,
        GPIO_Mode_Out_PP,
    };
    Motor motor_right = {
        &pwm_right,
        &gpio_direction_right_1,
        &gpio_direction_right_2,
    };
    right = &motor_right;
    info("motor_right DMP\r\n");
    Motor_Init(&motor_right);
    info("motor_right started\r\n");

    GPIO gpio_encoder_left = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_6 | GPIO_Pin_7,
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
    Encoder encoder_left = {
        &gpio_encoder_left,    &tim_left,
        &capture_left1,        &capture_left2,
        TIM_ICPolarity_Rising, TIM_ICPolarity_Falling,
    };
    info("starting encoder_left\r\n");
    Encoder_Init(&encoder_left);
    info("encoder_left started\r\n");

    GPIO gpio_encoder_right = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_6 | GPIO_Pin_7,
        GPIO_Mode_IPU,
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
    Encoder encoder_right = {
        &gpio_encoder_right,   &tim_right,
        &capture_right1,       &capture_right2,
        TIM_ICPolarity_Rising, TIM_ICPolarity_Rising,
    };
    info("starting encoder_right\r\n");
    Encoder_Init(&encoder_right);
    info("encoder_right started\r\n");

    PID stand = {
        ENABLE, DISABLE, ENABLE, STAND_KP, 0, STAND_KD, ZERO,
    };
    GlobalStand = &stand;

    TIM tim2 = {
        RCC_APB1Periph_TIM2, TIM2, TIM_InternalClock, 7200 - 1, 50 - 1, CMD,
    };
    info("starting TIM2\r\n");
    TIM_Init(&tim2, NULL);
    info("TIM2 started\r\n");

    TIM_Interrupt TIM_interrupt = {
        TIM2, TIM2_IRQn, NVIC_PriorityGroup_2, 0, 2,
    };
    info("starting TIM_Interrupt\r\n");
    TIM_Interrupt_Init(&TIM_interrupt);
    info("TIM_Interrupt started\r\n");

    info("started successfully\r\n");

    for (;;) {
    }
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {

        DMP_GetData(&pitch, &roll, &yaw);
        MPU_GetData(GlobalMPU, &xacc, &yacc, &zacc, &xgyro, &ygyro, &zgyro);

        speed_left = (int16_t)TIM_GetCounter(TIM3);
        TIM_SetCounter(TIM3, 0);

        speed_right = (int16_t)TIM_GetCounter(TIM4);
        TIM_SetCounter(TIM4, 0);

        if (WatchState) {
            WatchHandler(GlobalSerial);
        }

        int16_t stand = PID_StandCaculate(GlobalStand, pitch, ygyro);

        pulse_left = stand;
        pulse_right = stand;

        Motor_SetSpeed(left, pulse_left);
        Motor_SetSpeed(right, pulse_right);

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

int16_t PID_StandCaculate(PID *pid, float pitch, int16_t ygyro) {
    float error = pid->goal - pitch;
    return pid->Kp * error + pid->Kd * ygyro;
}

void ReceiveHandler(Serial *serial) {
    if (serial->RecieveFlag == SET) {
        switch (serial->type) {
        case Byte:
            Serial_SendString(serial, "\r");
            info("received Byte [%d]\r\n", serial->ByteData);

            if (serial->ByteData == 0x0D) {
                WatchState = DISABLE;
            }
            break;

        case HexPack:
            break;

        case StringPack:
            info("receive command [>%s]\r\n", serial->StringData);

            char *Head = strtok(serial->StringData, " ");
            if (strcmp(Head, "set") == 0) {
                char *Body = strtok(NULL, " ");

                if (strcmp(Body, "zero") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%f", &GlobalStand->goal);
                    info("successfully set %s to %f\r\n", Body,
                         GlobalStand->goal);
                } else if (strcmp(Body, "kp") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%f", &GlobalStand->Kp);
                    info("successfully set %s to %f\r\n", Body,
                         GlobalStand->Kp);
                } else if (strcmp(Body, "kd") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%f", &GlobalStand->Kd);
                    info("successfully set %s to %f\r\n", Body,
                         GlobalStand->Kd);
                } else {
                    error("unknow command\r\n");
                }

            } else if (strcmp(Head, "call") == 0) {

            } else if (strcmp(Head, "show") == 0) {
                char *Body = strtok(NULL, " ");

                if (strcmp(Body, "zero") == 0) {
                    info("%s: %f\r\n", Body, GlobalStand->goal);
                } else if (strcmp(Body, "kp") == 0) {
                    info("%s: %f\r\n", Body, GlobalStand->Kp);
                } else if (strcmp(Body, "kd") == 0) {
                    info("%s: %f\r\n", Body, GlobalStand->Kd);
                } else {
                    error("unknow command\r\n");
                }

            } else if (strcmp(Head, "watch") == 0) {
                WatchState = ENABLE;

            } else if (strcmp(Head, "reset") == 0) {
                info("call reset succeeeded\r\n");
                NVIC_SystemReset();
            } else {
                error("unknow command\r\n");
            }
            break;

        default:
            break;
        }

        Serial_Clear(serial);
    }
}

void WatchHandler(Serial *serial) {
    Serial_SendString(serial, "\r");
    Serial_SendString(serial, "pitch %+5.1f| ", pitch);
    Serial_SendString(serial, "ygyro %+5d| ", ygyro - ygyro_offset);
    Serial_SendString(serial, "speed_left %+4hd| ", speed_left);
    Serial_SendString(serial, "speed_right %+4hd| ", speed_right);
    // Serial_SendString(serial, "\r\n");
}