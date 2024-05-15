#ifndef __RTC_H
#define __RTC_H

#include <stdint.h>

void RTC_Init(void);
uint32_t RTC_time_s(void);
uint32_t RTC_time_ms(void);

#endif