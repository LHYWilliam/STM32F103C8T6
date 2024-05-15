#include "stm32f10x.h"
#include "stm32f10x_rtc.h"
#include <stdint.h>

#include "rtc.h"

void RTC_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);

    PWR_BackupAccessCmd(ENABLE);

    RCC_LSEConfig(RCC_LSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != SET)
        ;
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    RCC_RTCCLKCmd(ENABLE);

    RTC_WaitForSynchro();
    RTC_WaitForLastTask();

    RTC_SetPrescaler(32768 - 1);
    RTC_WaitForLastTask();

    // RCC_LSICmd(ENABLE);
    // while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) != SET)
    //     ;

    // RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
    // RCC_RTCCLKCmd(ENABLE);

    // RTC_WaitForSynchro();
    // RTC_WaitForLastTask();

    // RTC_SetPrescaler(40000 - 1);
    // RTC_WaitForLastTask();

    RTC_SetCounter(0);
    RTC_WaitForLastTask();
}

uint32_t RTC_time(void) { return RTC_GetCounter(); }