#include "stm32f10x.h"

#include "adc.h"
#include "dma.h"
#include "encoder.h"
#include "motor.h"
#include "oled.h"
#include "pid.h"
#include "rtc.h"
#include "serial.h"
#include "tim.h"
#include <stdint.h>

Motor motorLeft = {
    .PWM = "A11",
    .IN1 = "B12",
    .IN2 = "B13",
    .TIMx = TIM1,
    .channel = "4",
    .TIM_Init = ENABLE,
    .invert = DISABLE,
};

Motor motorRight = {
    .PWM = "A8",
    .IN1 = "B14",
    .IN2 = "B15",
    .TIMx = TIM1,
    .channel = "1",
    .TIM_Init = DISABLE,
    .invert = DISABLE,
};

Encoder encoderLeft = {
    .gpio = "A6 | A7",
    .TIMx = TIM3,
    .invert = DISABLE,
};

Encoder encoderRight = {
    .gpio = "B6 | B7",
    .TIMx = TIM4,
    .invert = ENABLE,
};

ADC adc = {
    .ADCx = ADC1,
    .gpio = "A0 | A1 | A2",
    .channel = "0 | 1 | 2",
    .DMA = ENABLE,
};

uint16_t infraredValue[3];
DMA dma = {
    .DMAx = DMA1,
    .channel = 1,

    .sourceAddr = (uint32_t)&ADC1->DR,
    .sourceInc = DISABLE,

    .targetAddr = (uint32_t)infraredValue,
    .targetInc = ENABLE,

    .DataSize = 16,
    .BufferSize = 3,
    .Circular = ENABLE,
    .M2M = DISABLE,
};

PID tracePID = {
    .Kp = -0.12,
    .Ki = 0,
    .Kd = -0.0045,
    .imax = 1024,
};

PID motorLeftPID = {
    .Kp = -4.096,
    .Ki = -81.92,
    .Kd = 0,
    .imax = 2048,
};

PID motorRightPID = {
    .Kp = -4.096,
    .Ki = -81.92,
    .Kd = 0,
    .imax = 2048,
};

Timer timer = {
    .TIMx = TIM2,
    .ms = 10,
};

typedef enum {
    Left,
    Center,
    Right,
} InfraredType;

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
DirectionType direction = TurnRight;
char *directionString[] = {"Forward", "TurnLeft", "TurnRight", "TurnBack"};

typedef enum {
    OffLine = 1024,
    OnLine = 3800,
    OnCross,
} CrossType;
CrossType line = OffLine;
char *crossString[] = {"NoCross", "InCross", "PassCross"};

Serial *GlobalSerial;

uint16_t infraredMax = 3850, infraredMaxCenter = 2500;
int16_t infraredLeft = 0, infraredCenter = 0, infraredRight = 0;
int16_t tracePIDError = 0;

float encoderToPWM = 7200. / 140.;

int16_t AdvancediffSpeed = 0;
uint16_t advanceBaseSpeed = 1024;

int16_t turnDiffSpeed = 0;
uint16_t turnBaseSpeed = 390;

uint16_t turnTime = 0;
uint16_t turnBaseTime = 1000;
uint16_t turnTimer = DISABLE;

int16_t speedLfet = 0, speedRight = 0;
int16_t leftPIDOut = 0, rightPIDOut = 0;

void Serial_Praser(Serial *serial);
void Serial_Handler(Serial *serial);

int main() {
    RTC_Init();
    OLED_Init();

    Motor_Init(&motorLeft);
    Motor_Init(&motorRight);

    Encoder_Init(&encoderLeft);
    Encoder_Init(&encoderRight);

    ADC_Init_(&adc);
    DMA_Init_(&dma);
    DMA_Start(&dma);
    ADC_Start(&adc);

    PID_Init(&tracePID);
    PID_Init(&motorLeftPID);
    PID_Init(&motorRightPID);

    Timer_Init(&timer);

    int16_t leftMax = 0, leftMaxCenter = 0, rightMax = 0, rightMaxCenter = 0;
    for (;;) {
        // OLED_ShowString(1, 1, "Action:         ");
        // OLED_ShowString(1, 8,
        //                 action == Turn ? directionString[direction]
        //                                : actionString[action]);
        // OLED_ShowSignedNum(2, 6, AdvancediffSpeed, 5);
        // OLED_ShowSignedNum(3, 6, leftPIDOut, 5);
        // OLED_ShowSignedNum(4, 7, rightPIDOut, 5);

        // Serial_SendString(&serial, "%d,%d,%d,%d\n", leftPIDOut, rightPIDOut,
        //                   (int16_t)(speedLfet * encoderToPWM),
        //                   (int16_t)(speedRight * encoderToPWM));

        OLED_ShowString(1, 1, "Left  :     ");
        OLED_ShowString(2, 1, "Center:     ");
        OLED_ShowString(3, 1, "Right :    ");
        OLED_ShowNum(1, 8, infraredValue[Left], 4);
        OLED_ShowNum(2, 8, infraredValue[Center], 4);
        OLED_ShowNum(3, 8, infraredValue[Right], 4);

        // OLED_ShowString(1, 1, "LeftMax  :     ");
        // OLED_ShowString(2, 1, "MaxCenter:     ");
        // OLED_ShowString(3, 1, "RightMax :    ");
        // OLED_ShowString(4, 1, "MaxCenter :    ");
        // OLED_ShowNum(1, 11, leftMax, 4);
        // OLED_ShowNum(2, 11, leftMaxCenter, 4);
        // OLED_ShowNum(3, 11, rightMax, 4);
        // OLED_ShowNum(4, 11, rightMaxCenter, 4);

        // OLED_ShowString(1, 1, "Error:     ");
        // OLED_ShowString(2, 1, "  Out:       ");
        // OLED_ShowSignedNum(1, 8, tracePIDError, 6);
        // OLED_ShowSignedNum(1, 8, AdvancediffSpeed, 6);
    }
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        infraredLeft = infraredValue[Left];
        infraredCenter = infraredValue[Center];
        infraredRight = infraredValue[Right];
        speedLfet = Encoder_Get(&encoderLeft);
        speedRight = Encoder_Get(&encoderRight);

        switch (line) {
        case OffLine:
            if (infraredLeft > OffLine || infraredCenter > OffLine ||
                infraredRight > OffLine) {
                line = OnLine;
                action = Advance;
            }
            break;

        case OnLine:
            if (infraredLeft < OffLine && infraredCenter < OffLine &&
                infraredRight < OffLine) {
                line = OffLine;
                action = Round;
            }
            if (infraredLeft > OnCross && infraredCenter > OnCross &&
                infraredRight > OnCross) {
                if (direction != Forward) {
                    line = OnCross;
                    action = Turn;
                }
            }
            break;

        case OnCross:
            if (turnTimer == DISABLE) {
                switch (direction) {
                case TurnLeft:
                    turnDiffSpeed = -turnBaseSpeed;
                    turnTime = turnBaseTime;
                    break;

                case TurnRight:
                    turnDiffSpeed = turnBaseSpeed;
                    turnTime = turnBaseTime;
                    break;

                case TurnBack:
                    turnDiffSpeed = turnBaseSpeed;
                    turnTime = 2 * turnBaseTime;
                    break;
                default:
                    break;
                }
                turnTimer = ENABLE;
            }
            if (turnTimer > turnTime) {
                line = OnLine;
                action = Advance;
                turnTimer = DISABLE;
            }
            break;

        default:
            break;
        }

        switch (action) {
        case Stop:
            leftPIDOut =
                PID_Caculate(&motorLeftPID, speedLfet * encoderToPWM - 0);
            rightPIDOut =
                PID_Caculate(&motorRightPID, speedRight * encoderToPWM - 0);

            Motor_Set(&motorLeft, leftPIDOut);
            Motor_Set(&motorRight, rightPIDOut);
            break;

        case Advance:
            if (infraredCenter > infraredMaxCenter) {
                tracePIDError = infraredLeft - infraredRight;
            } else if (infraredLeft > infraredRight) {
                tracePIDError =
                    (2 * infraredMax - infraredLeft) - infraredRight;
            } else if (infraredLeft < infraredRight) {
                tracePIDError =
                    infraredLeft - (2 * infraredMax - infraredRight);
            }
            AdvancediffSpeed = PID_Caculate(&tracePID, tracePIDError);

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

                Motor_Set(&motorLeft, 3072 + leftPIDOut);
                Motor_Set(&motorRight, 3072 + rightPIDOut);

                turnTimer += 10;
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