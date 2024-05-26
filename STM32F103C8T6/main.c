#include "controller.h"
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
#include "pwm.h"
#include "rtc.h"
#include "serial.h"
#include "tim.h"
#include "usart.h"

#define MPU6050_DEVICE_ADDRESS ((uint8_t)0x68)

I2C *GlobalI2C;
Serial *GlobalSerial;
Controller *GlobalController;

uint8_t WatchState = DISABLE;

void ReceiveHandler(Serial *serial, Controller *controller);
void WatchHandler(Controller *controller, Serial *serial);

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

    int16_t speed_left, speed_right;
    float pitch = 0, roll = 0, yaw = 0;
    int8_t pulse_left = 0, pulse_right = 0;

    Controller controller;
    GlobalController = &controller;
    info("starting Controller\r\n");
    Controller_Init(&controller);
    info("Controller started\r\n");

    Controller_Add(&controller, "pitch", &pitch, DATA);
    Controller_Add(&controller, "roll", &roll, DATA);
    Controller_Add(&controller, "yaw", &yaw, DATA);
    Controller_Add(&controller, "speed_left", &speed_left, DATA);
    Controller_Add(&controller, "speed_right", &speed_right, DATA);
    Controller_Add(&controller, "pulse_left", &pulse_left, DATA);
    Controller_Add(&controller, "pulse_right", &pulse_right, DATA);

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
    info("starting MPU\r\n");
    MPU_Init(&mpu);
    info("MPU started\r\n");

    info("starting DMP\r\n");
    DMP_Init();
    info("DMP started\r\n");

    TIM tim_pwm = {
        RCC_APB2Periph_TIM1, TIM1, TIM_InternalClock, 720 - 1, 100 - 1, CMD,
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
    info("motor_right DMP\r\n");
    Motor_Init(&motor_right);
    info("motor_right started\r\n");

    TIM tim2 = {
        RCC_APB1Periph_TIM2, TIM2, TIM_InternalClock, 7200 - 1, 100 - 1, CMD,
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

    info("started successfully\r\n");

    for (;;) {
        Motor_SetSpeed(&motor_left, pulse_left);
        Motor_SetSpeed(&motor_right, pulse_right);
    }
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {

        *(int16_t *)Controller_Eval(GlobalController, "speed_left", DATA) =
            (int16_t)TIM_GetCounter(TIM3);
        TIM_SetCounter(TIM3, 0);

        *(int16_t *)Controller_Eval(GlobalController, "speed_right", DATA) =
            (int16_t)TIM_GetCounter(TIM4);
        TIM_SetCounter(TIM4, 0);

        DMP_GetData((float *)Controller_Eval(GlobalController, "pitch", DATA),
                    (float *)Controller_Eval(GlobalController, "roll", DATA),
                    (float *)Controller_Eval(GlobalController, "yaw", DATA));

        if (WatchState) {
            WatchHandler(GlobalController, GlobalSerial);
        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

void USART3_IRQHandler(void) {
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {

        Serial_Parse(GlobalSerial);
        ReceiveHandler(GlobalSerial, GlobalController);

        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}

void ReceiveHandler(Serial *serial, Controller *controller) {
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

                if (strcmp(Body, "pulse_left") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%hhd",
                           (int8_t *)Controller_Eval(controller, "pulse_left",
                                                     DATA));
                    info("successfully set %s to %hhd\r\n", Body,
                         *(int8_t *)Controller_Eval(controller, "pulse_left",
                                                    DATA));
                } else if (strcmp(Body, "pulse_right") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(input, "%hhd",
                           (int8_t *)Controller_Eval(controller, "pulse_right",
                                                     DATA));
                    info("successfully set %s to %hhd\r\n", Body,
                         *(int8_t *)Controller_Eval(controller, "pulse_right",
                                                    DATA));
                } else {
                    error("unknow command\r\n");
                }

            } else if (strcmp(Head, "call") == 0) {

            } else if (strcmp(Head, "show") == 0) {
                char *Body = strtok(NULL, " ");

                if (strcmp(Body, "pulse_left") == 0) {
                    info("%s: %d\r\n", Body,
                         *(int8_t *)Controller_Eval(controller, "pulse_left",
                                                    DATA));
                } else if (strcmp(Body, "pulse_right") == 0) {
                    info("%s: %d\r\n", Body,
                         *(int8_t *)Controller_Eval(controller, "pulse_right",
                                                    DATA));
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

void WatchHandler(Controller *controller, Serial *serial) {
    Serial_SendString(serial, "pitch %+.1f ",
                      *(float *)Controller_Eval(controller, "pitch", DATA));
    Serial_SendString(
        serial, "speed_left %+hd ",
        *(int16_t *)Controller_Eval(controller, "speed_left", DATA));
    Serial_SendString(
        serial, "speed_right %+hd ",
        *(int16_t *)Controller_Eval(controller, "speed_right", DATA));

    Serial_SendString(serial, "\r\n");
}