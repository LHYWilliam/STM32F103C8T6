#include "controller.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

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
LED *GlobalLED;

uint8_t count = 0;
uint8_t RecieveFlag = RESET;

PackType type = None;

uint8_t byte;
uint8_t HexData[32];
char StringData[32];

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
    info("starting Controller interrupt\r\n");
    Controller_Init(&controller);
    info("Controller started\r\n");

    Controller_Add(&controller, "LED_Turn", LED_Turn, FUNCTION);

    for (;;) {
        if (RecieveFlag == SET) {
            if (type == ByteData) {
                info("received %c\r\n", byte);
            } else if (type == HexPack) {
            } else if (type == StringPack) {
                info("received %s\r\n", StringData);
                char *goal = strtok(StringData, " ");

                if (strcmp(goal, "call") == 0) {
                    goal = strtok(NULL, " ");

                    if (strcmp(goal, "LED_Turn") == 0) {
                        ((void (*)(LED *))Controller_Eval(&controller, goal,
                                                          FUNCTION))(&led);
                    }
                }
            }

            count = 0;
            type = None;
            RecieveFlag = RESET;
        }
    }
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
        byte = USART_ReceiveData(USART1);

        if (type == None) {
            if (byte == 0xFF) {
                type = HexPack;
            } else if (byte == '>') {
                type = StringPack;
            } else {
                type = ByteData;
                RecieveFlag = SET;
            }
        } else if (type == HexPack) {
            if (byte == 0xFE) {
                RecieveFlag = SET;
            } else {
                HexData[count++] = byte;
            }
        } else if (type == StringPack) {
            if (count >= 1 && byte == '\n' && StringData[count - 1] == '\r') {
                StringData[--count] = '\0';

                RecieveFlag = SET;
            } else {
                StringData[count++] = byte;
            }
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}