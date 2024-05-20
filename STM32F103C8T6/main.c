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

Serial *GlobalSerial;
I2C *GlobalI2C;

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
    info("starting LED interrupt\r\n");
    LED_Init(&led);
    info("LED started\r\n");

    Controller controller;
    info("starting Controller interrupt\r\n");
    Controller_Init(&controller);
    info("Controller started\r\n");

    uint8_t temp = 114;
    Controller_Add(&controller, "temp", &temp, DATA);
    Controller_Add(&controller, "LED_Turn", LED_Turn, FUNCTION);

    for (;;) {
        if (serial.RecieveFlag == SET) {
            if (serial.type == Byte) {
                info("received %c\r\n", Byte);
            } else if (serial.type == HexPack) {
            } else if (serial.type == StringPack) {
                info("received %s\r\n", serial.StringData);
                char *goal = strtok(serial.StringData, " ");
                if (strcmp(goal, "set") == 0) {
                    goal = strtok(NULL, " ");
                    if (strcmp(goal, "temp")) {
                        uint8_t *data =
                            Controller_Eval(&controller, goal, DATA);
                        goal = strtok(NULL, " ");
                        sscanf(goal, "%hhu", data);

                        info("set %s to %d", goal, *data);
                    }

                } else if (strcmp(goal, "call") == 0) {
                    goal = strtok(NULL, " ");

                    if (strcmp(goal, "LED_Turn") == 0) {
                        ((void (*)(LED *))Controller_Eval(&controller, goal,
                                                          FUNCTION))(&led);
                        info("call %s succeeeded\r\n", goal);
                    }
                } else if (strcmp(goal, "show") == 0) {
                    goal = strtok(NULL, " ");

                    info("%s: %d\r\n", goal,
                         *(uint8_t *)Controller_Eval(&controller, goal, DATA))
                }
            }

            serial.count = 0;
            serial.type = None;
            serial.RecieveFlag = RESET;
        }
    }
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {

        Serial_Parse(GlobalSerial);

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}