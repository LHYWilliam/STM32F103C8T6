#include "controller.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gpio.h"
#include "i2c.h"
#include "interrupt.h"
#include "led.h"
#include "rtc.h"
#include "serial.h"
#include "usart.h"

#define MPU6050_DEVICE_ADDRESS ((uint8_t)0x68)

LED *GlobalLED;
I2C *GlobalI2C;
Serial *GlobalSerial;
Controller *GlobalController;

void SerialReceive_Handler(void);

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
    info("Serial started\r\n");

    USART_Interrupt interrupt = {
        USART1, USART_IT_RXNE, USART1_IRQn, NVIC_PriorityGroup_2, 1, 1,
    };
    info("starting USART interrupt\r\n");
    USART_Interrupt_Init(&interrupt);
    info("USART interrupt started\r\n");

    GPIO gpio = {
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_0,
        GPIO_Mode_Out_PP,
    };
    LED led = {
        &gpio,
        HIGH,
    };
    GlobalLED = &led;
    info("starting LED interrupt\r\n");
    LED_Init(&led);
    info("LED started\r\n");

    Controller controller;
    GlobalController = &controller;
    info("starting Controller interrupt\r\n");
    Controller_Init(&controller);
    info("Controller started\r\n");

    float temp = 114.;
    Controller_Add(&controller, "temp", &temp, DATA);
    Controller_Add(&controller, "LED_Turn", LED_Turn, FUNCTION);

    for (;;) {
        SerialReceive_Handler();
    }
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {

        Serial_Parse(GlobalSerial);

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void SerialReceive_Handler(void) {
    if (GlobalSerial->RecieveFlag == SET) {
        switch (GlobalSerial->type) {
        case Byte:
            info("received %c\r\n", GlobalSerial->ByteData);
            break;

        case HexPack:
            break;

        case StringPack:
            info("received %s\r\n", GlobalSerial->StringData);

            char *goal = strtok(GlobalSerial->StringData, " ");
            if (strcmp(goal, "set") == 0) {
                goal = strtok(NULL, " ");
                if (strcmp(goal, "temp") == 0) {
                    float *data = Controller_Eval(GlobalController, goal, DATA);
                    goal = strtok(NULL, " ");
                    sscanf(goal, "%f", data);

                    info("set temp to %f\r\n", *data);
                }

            } else if (strcmp(goal, "call") == 0) {
                goal = strtok(NULL, " ");

                if (strcmp(goal, "LED_Turn") == 0) {
                    ((void (*)(LED *))Controller_Eval(GlobalController, goal,
                                                      FUNCTION))(GlobalLED);
                    info("call %s succeeeded\r\n", goal);
                }
            } else if (strcmp(goal, "show") == 0) {
                goal = strtok(NULL, " ");

                info("%s: %f\r\n", goal,
                     *(float *)Controller_Eval(GlobalController, goal, DATA))
            }
            break;

        default:
            break;
        }

        GlobalSerial->count = 0;
        GlobalSerial->type = None;
        GlobalSerial->RecieveFlag = RESET;
    }
}