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
#include "rtc.h"
#include "serial.h"
#include "usart.h"

I2C *GlobalI2C;
Serial *GlobalSerial;
Controller *GlobalController;

void Serial_ReceiveHandler(Serial *serial, Controller *controller);
void Controller_WatchHandler(Controller *controller, Serial *serial);

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

    Controller controller;
    GlobalController = &controller;
    info("starting Controller\r\n");
    Controller_Init(&controller);
    info("Controller started\r\n");

    uint8_t WatchState = DISABLE;
    uint8_t temp = 114;
    Controller_Add(&controller, "temp", &temp, DATA);
    Controller_Add(&controller, "WatchState", &WatchState, DATA);

    for (;;) {
        if (WatchState) {
            Controller_WatchHandler(&controller, &serial);
        }
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
                *(uint8_t *)Controller_Eval(controller, "WatchState", DATA) =
                    DISABLE;
            }
            break;

        case HexPack:
            break;

        case StringPack:
            info("receive command [>%s]\r\n", serial->StringData);

            char *Head = strtok(serial->StringData, " ");
            if (strcmp(Head, "set") == 0) {
                char *Body = strtok(NULL, " ");

                if (strcmp(Body, "temp") == 0) {
                    char *input = strtok(NULL, " ");
                    sscanf(
                        input, "%hhu",
                        (uint8_t *)Controller_Eval(controller, "temp", DATA));
                    info("successfully set %s to %d\r\n", Body,
                         *(uint8_t *)Controller_Eval(controller, "temp", DATA));
                } else {
                    error("unknow command\r\n");
                }

            } else if (strcmp(Head, "call") == 0) {
            } else if (strcmp(Head, "show") == 0) {
                char *Body = strtok(NULL, " ");

                if (strcmp(Body, "temp") == 0) {
                    info("%s: %d\r\n", Body,
                         *(uint8_t *)Controller_Eval(controller, Body, DATA))
                } else {
                    error("unknow command\r\n");
                }

            } else if (strcmp(Head, "watch") == 0) {
                *(uint8_t *)Controller_Eval(controller, "WatchState", DATA) =
                    ENABLE;

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
    Serial_SendString(serial, "temp: %d ",
                      *(uint8_t *)Controller_Eval(controller, "temp", DATA));
}