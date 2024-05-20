#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

#include "gpio.h"
#include "rtc.h"
#include "usart.h"

#define info(args...)                                                          \
    {                                                                          \
        Serial_SendString(GlobalSerial, "[INFO][Time %ds] ", RTC_time_s());    \
        Serial_SendString(GlobalSerial, args);                                 \
    }

#define error(args...)                                                         \
    {                                                                          \
        Serial_SendString(GlobalSerial, "[ERROR][Time %ds] ", RTC_time_s());   \
        Serial_SendString(GlobalSerial, args);                                 \
    }

typedef enum {
    None,
    Byte,
    HexPack,
    StringPack,
} PackType;

typedef struct {
    GPIO *TX;
    GPIO *RX;
    USART *usart;

    uint8_t count;
    uint8_t RecieveFlag;

    PackType type;

    uint8_t ByteData;
    uint8_t HexData[32];
    char StringData[32];
} Serial;

void Serial_Init(Serial *serial);
void Serial_SendByte(Serial *serial, uint8_t byte);
void Serial_SendHex(Serial *serial, uint8_t byte);
void Serial_SendString(Serial *serial, char *format, ...);
void Serial_SendHexPack(Serial *serial, uint8_t *array, uint16_t length);
void Serial_SendStringPack(Serial *serial, char *string);
void Serial_Parse(Serial *serial);

#endif