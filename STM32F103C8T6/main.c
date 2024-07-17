#include "stm32f10x.h"
#include "stm32f10x_tim.h"

#include <stdint.h>

#include "delay.h"
#include "i2c.h"
#include "motor.h"
#include "oled.h"
#include "serial.h"
#include "tim.h"

Serial serial = {
    .TX = "B10",
    .RX = "B11",
    .USARTx = USART3,
    .Interrupt = ENABLE,
    .DMA = DISABLE,
};

Motor motorLeft = {
    .PWM = "A11",
    .IN1 = "B12",
    .IN2 = "B13",
    .TIMx = TIM1,
    .channel = 4,
    .TIM_Init_Mode = ENABLE,
    .invert = DISABLE,
};

Motor motorRight = {
    .PWM = "A8",
    .IN1 = "B14",
    .IN2 = "B15",
    .TIMx = TIM1,
    .channel = 1,
    .TIM_Init_Mode = DISABLE,
    .invert = DISABLE,
};

Timer timer = {
    .TIMx = TIM2,
    .ms = 10,
};

typedef enum {
    Stop,
    Advance,
    Turn,
    Round,
} ActionType;
ActionType action = Stop;

typedef enum {
    Forward,
    TurnLeft,
    TurnRight,
    TurnBack,
} DirectionType;
DirectionType direction = Forward;

I2C *GlobalI2C;
Serial *GlobalSerial = &serial;

uint16_t advanceBaseSpeed = 1200;
uint16_t turnBaseSpeed = 1050;

int16_t AdvancediffSpeed = 0;
int16_t turnDiffSpeed = 0;

uint16_t turnTimer = DISABLE;
uint16_t turnBaseTime = 1000;
uint16_t turnTime = 0;

void Serial_Praser(Serial *serial);
void Serial_Handler(Serial *serial);

int main() {
    Delay_ms(2500);
    OLED_Init();

    Serial_Init(&serial);

    Motor_Init(&motorLeft);
    Motor_Init(&motorRight);

    Timer_Init(&timer);

    for (;;) {
        OLED_ShowNum(1, 1, action, 1);
        OLED_ShowSignedNum(2, 1, AdvancediffSpeed, 5);
        OLED_ShowSignedNum(3, 1, advanceBaseSpeed + AdvancediffSpeed, 5);
        OLED_ShowSignedNum(4, 1, advanceBaseSpeed - AdvancediffSpeed, 5);
    }
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {

        switch (action) {
        case Stop:
            Motor_Set(&motorLeft, 0);
            Motor_Set(&motorRight, 0);
            break;

        case Advance:
            Motor_Set(&motorLeft, advanceBaseSpeed + AdvancediffSpeed);
            Motor_Set(&motorRight, advanceBaseSpeed - AdvancediffSpeed);
            break;

        case Turn:
            if (turnTimer) {
                Motor_Set(&motorLeft, +turnDiffSpeed);
                Motor_Set(&motorRight, -turnDiffSpeed);

                turnTimer += 10;
                if (turnTimer > turnTime) {
                    action = Stop;
                    turnTimer = DISABLE;
                }
            }
            break;

        case Round:
            Motor_Set(&motorLeft, +turnDiffSpeed);
            Motor_Set(&motorRight, -turnDiffSpeed);
            break;
        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

void USART3_IRQHandler(void) {
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {

        Serial_Praser(&serial);
        Serial_Handler(&serial);

        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}

void Serial_Praser(Serial *serial) {
    serial->ByteData = USART_ReceiveData(serial->USARTx);

    switch (serial->type) {
    case None:
        if (serial->ByteData == 0xFF) {
            serial->type = HexPack;
        } else {
            Serial_Clear(serial);
        }
        break;

    case HexPack:
        if (serial->ByteData == 0xFE && serial->count == 3) {
            serial->RecieveFlag = SET;
        } else {
            serial->HexData[serial->count++] = serial->ByteData;
        }

        if (serial->count > 3) {
            Serial_Clear(serial);
        }
        break;

    default:
        Serial_Clear(serial);
        break;
    }
}

void Serial_Handler(Serial *serial) {
    if (serial->RecieveFlag == SET) {
        action = serial->HexData[0];

        switch (action) {
        case Stop:
            break;

        case Advance:
            AdvancediffSpeed =
                (int16_t)(serial->HexData[1] << 8 | serial->HexData[2]);
            break;

        case Turn:
            direction = (int16_t)(serial->HexData[1] << 8 | serial->HexData[2]);

            switch (direction) {
            case Forward:
                break;

            case TurnLeft:
                turnTime = turnBaseTime;
                turnDiffSpeed = -turnBaseSpeed;
                break;

            case TurnRight:
                turnTime = turnBaseTime;
                turnDiffSpeed = turnBaseSpeed;
                break;

            case TurnBack:
                turnTime = turnBaseTime * 2;
                turnDiffSpeed = turnBaseSpeed;
                break;
            }

            turnTime = turnTime > 0 ? turnTime : -turnTime;
            turnTimer = ENABLE;
            break;

        case Round:
            break;

        default:
            action = Stop;
            break;
        }

        Serial_Clear(serial);
    }
}