#include "controller.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "capture.h"
#include "dmp.h"
#include "encoder.h"
#include "gpio.h"
#include "i2c.h"
#include "interrupt.h"
#include "mpu.h"
#include "rtc.h"
#include "serial.h"
#include "usart.h"

#define MPU6050_DEVICE_ADDRESS ((uint8_t)0x68)

I2C *GlobalI2C;
Serial *GlobalSerial;
Controller *GlobalController;

void Serial_ReceiveHandler(Serial *serial, Controller *controller);
void Controller_WatchHandler(Controller *controller, Serial *serial);

uint8_t WatchState = DISABLE;

int16_t speed_left, speed_right;
float roll = 0, pitch = 0, yaw = 0;

int main() {
    RTC_Init();

    GPIO TX = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_9,
        GPIO_Mode_AF_PP,
    };
    GPIO RX = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_10,
        GPIO_Mode_IPU,
    };
    USART usart = {
        RCC_APB2Periph_USART1,
        USART1,
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
        USART1, USART_IT_RXNE, USART1_IRQn, NVIC_PriorityGroup_2, 1, 1,
    };
    info("starting USART interrupt\r\n");
    USART_Interrupt_Init(&interrupt);
    info("USART interrupt started\r\n");

    TIM tim2 = {
        RCC_APB1Periph_TIM2, TIM2, TIM_InternalClock, 7200 - 1, 1000 - 1, CMD,
    };
    TIM_Init(&tim2, NULL);

    TIM_Interrupt TIM_interrupt = {
        TIM2, TIM2_IRQn, NVIC_PriorityGroup_2, 1, 1,
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
        TIM_ICPolarity_Rising, TIM_ICPolarity_Rising,
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

    GPIO SCL = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_10,
        GPIO_Mode_Out_OD,
    };
    GPIO SDA = {
        RCC_APB2Periph_GPIOB,
        GPIOB,
        GPIO_Pin_11,
        GPIO_Mode_Out_OD,
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

    Controller controller;
    GlobalController = &controller;
    info("starting Controller\r\n");
    Controller_Init(&controller);
    info("Controller started\r\n");

    Controller_Add(&controller, "pitch", &pitch, DATA);
    Controller_Add(&controller, "speed_left", &speed_left, DATA);
    Controller_Add(&controller, "speed_right", &speed_right, DATA);

    for (;;) {
        if (WatchState) {
            Controller_WatchHandler(&controller, &serial);
        }
        DMP_GetData(&pitch, &roll, &yaw);
    }
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {

        speed_left = (int16_t)TIM_GetCounter(TIM3);
        TIM_SetCounter(TIM3, 0);

        speed_right = (int16_t)TIM_GetCounter(TIM4);
        TIM_SetCounter(TIM4, 0);

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {

        Serial_Parse(GlobalSerial);
        Serial_ReceiveHandler(GlobalSerial, GlobalController);

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void Serial_ReceiveHandler(Serial *serial, Controller *controller) {
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

                if (strcmp(Body, "speed") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(
                        input, "%hd",
                        (int16_t *)Controller_Eval(controller, "speed", DATA));
                    info(
                        "successfully set %s to %hd\r\n", Body,
                        *(int16_t *)Controller_Eval(controller, "speed", DATA));
                } else {
                    error("unknow command\r\n");
                }

            } else if (strcmp(Head, "call") == 0) {

            } else if (strcmp(Head, "show") == 0) {
                char *Body = strtok(NULL, " ");

                if (strcmp(Body, "speed") == 0) {
                    info("%s: %hd\r\n", Body,
                         *(int16_t *)Controller_Eval(controller, Body, DATA));
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

void Controller_WatchHandler(Controller *controller, Serial *serial) {
    Serial_SendString(serial, "\r[WATCH][Time %ds] ", RTC_time_s());
    Serial_SendString(serial, "pitch: %+6.1f ",
                      *(float *)Controller_Eval(controller, "pitch", DATA));
    Serial_SendString(
        serial, "speed_left: %+5hd ",
        *(int16_t *)Controller_Eval(controller, "speed_left", DATA));
    Serial_SendString(
        serial, "speed_right: %+5hd ",
        *(int16_t *)Controller_Eval(controller, "speed_right", DATA));
}