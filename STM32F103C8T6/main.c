#include "stm32f10x.h"
#include "stm32f10x_tim.h"

#include <stdint.h>

#include "delay.h"
#include "encoder.h"
#include "i2c.h"
#include "motor.h"
#include "oled.h"
#include "pid.h"
#include "rtc.h"
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

Encoder encoderLeft = {
    .gpio = "A6 | A7",
    .TIMx = TIM3,
    .invert = ENABLE,
};

Encoder encoderRight = {
    .gpio = "B6 | B7",
    .TIMx = TIM4,
    .invert = DISABLE,
};

PID motorLeftPID = {
    .Kp = -3.5,
    .Ki = -8,
    .Kd = 0.05,
    .imax = 1024,
};

PID motorRightPID = {
    .Kp = -2.5,
    .Ki = -36,
    .Kd = 0.05,
    .imax = 1024,
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
char *actionString[] = {"Stop", "Advance", "Turn", "Round"};

typedef enum {
    Forward,
    TurnLeft,
    TurnRight,
    TurnBack,
} DirectionType;
DirectionType direction = Forward;
char *directionString[] = {"Forward", "TurnLeft", "TurnRight", "TurnBack"};

I2C *GlobalI2C;
Serial *GlobalSerial = &serial;

float encoderToPWM = 96;

uint16_t advanceBaseSpeed = 1200;
uint16_t turnBaseSpeed = 1050;

int16_t AdvancediffSpeed = 0;
int16_t turnDiffSpeed = 0;

uint16_t turnTimer = DISABLE;
uint16_t turnBaseTime = 1000;
uint16_t turnTime = 0;

int16_t speedLfet = 0, speedRight = 0;
int16_t leftPIDOut = 0, rightPIDOut = 0;

void Serial_Praser(Serial *serial);
void Serial_Handler(Serial *serial);

int main() {
    RTC_Init();
    Delay_ms(2500);
    OLED_Init();

    Serial_Init(&serial);

    Motor_Init(&motorLeft);
    Motor_Init(&motorRight);

    Encoder_Init(&encoderLeft);
    Encoder_Init(&encoderRight);

    PID_Init(&motorLeftPID);
    PID_Init(&motorRightPID);

    Timer_Init(&timer);

    OLED_ShowString(2, 1, "Diff:     ");
    OLED_ShowString(3, 1, "Left:     ");
    OLED_ShowString(4, 1, "Right:    ");
    for (;;) {
        // OLED_ShowString(1, 1, "Action:         ");
        // OLED_ShowString(1, 8,
        //                 action == Turn ? directionString[direction]
        //                                : actionString[action]);
        // OLED_ShowSignedNum(2, 6, AdvancediffSpeed, 5);
        // OLED_ShowSignedNum(3, 6, leftPIDOut, 5);
        // OLED_ShowSignedNum(4, 7, rightPIDOut, 5);

        Serial_SendString(&serial, "%d,%d,%d,%d\n", leftPIDOut, rightPIDOut,
                          (int16_t)(speedLfet * encoderToPWM),
                          (int16_t)(speedRight * encoderToPWM));
    }
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        speedLfet = Encoder_Get(&encoderLeft);
        speedRight = Encoder_Get(&encoderRight);

        switch (action) {
        case Stop:
            leftPIDOut = 0, rightPIDOut = 0;

            Motor_Set(&motorLeft, leftPIDOut);
            Motor_Set(&motorRight, rightPIDOut);
            break;

        case Advance:
            leftPIDOut = PID_Caculate(
                &motorLeftPID, speedLfet * encoderToPWM -
                                   (advanceBaseSpeed + AdvancediffSpeed));
            rightPIDOut = PID_Caculate(
                &motorRightPID, speedRight * encoderToPWM -
                                    (advanceBaseSpeed - AdvancediffSpeed));
            LIMIT(leftPIDOut, -7200, 7200);
            LIMIT(rightPIDOut, -7200, 7200);

            Motor_Set(&motorLeft, leftPIDOut);
            Motor_Set(&motorRight, rightPIDOut);
            break;

        case Turn:
            if (turnTimer) {
                leftPIDOut = PID_Caculate(
                    &motorLeftPID, speedLfet * encoderToPWM - (+turnDiffSpeed));
                rightPIDOut =
                    PID_Caculate(&motorRightPID,
                                 speedRight * encoderToPWM - (-turnDiffSpeed));
                LIMIT(leftPIDOut, -7200, 7200);
                LIMIT(rightPIDOut, -7200, 7200);

                Motor_Set(&motorLeft, leftPIDOut);
                Motor_Set(&motorRight, rightPIDOut);

                turnTimer += 10;
                if (turnTimer > turnTime) {
                    action = Stop;
                    turnTimer = DISABLE;
                }
            }
            break;

        case Round:
            leftPIDOut = PID_Caculate(&motorLeftPID, speedLfet * encoderToPWM -
                                                         (+turnDiffSpeed));
            rightPIDOut = PID_Caculate(
                &motorRightPID, speedRight * encoderToPWM - (-turnDiffSpeed));
            LIMIT(leftPIDOut, -7200, 7200);
            LIMIT(rightPIDOut, -7200, 7200);

            Motor_Set(&motorLeft, leftPIDOut);
            Motor_Set(&motorRight, rightPIDOut);
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