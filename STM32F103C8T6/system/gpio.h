#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f10x.h"

#define BITBAND(addr, bitnum)                                                  \
    ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

#define GPIOA_ODR_Addr (GPIOA_BASE + 12)
#define GPIOB_ODR_Addr (GPIOB_BASE + 12)
#define GPIOC_ODR_Addr (GPIOC_BASE + 12)
#define GPIOD_ODR_Addr (GPIOD_BASE + 12)
#define GPIOE_ODR_Addr (GPIOE_BASE + 12)
#define GPIOF_ODR_Addr (GPIOF_BASE + 12)
#define GPIOG_ODR_Addr (GPIOG_BASE + 12)

#define GPIOA_IDR_Addr (GPIOA_BASE + 8)
#define GPIOB_IDR_Addr (GPIOB_BASE + 8)
#define GPIOC_IDR_Addr (GPIOC_BASE + 8)
#define GPIOD_IDR_Addr (GPIOD_BASE + 8)
#define GPIOE_IDR_Addr (GPIOE_BASE + 8)
#define GPIOF_IDR_Addr (GPIOF_BASE + 8)
#define GPIOG_IDR_Addr (GPIOG_BASE + 8)

#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n)
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n)
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n)
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n)
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n)
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n)
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n)
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)

#define RCC_APB2Periph_GPIOx(x)                                                \
    (x[0]) == 'A' ? RCC_APB2Periph_GPIOA : RCC_APB2Periph_GPIOB

#define GPIOx(x) (x[0]) == 'A' ? GPIOA : GPIOB

#define GPIO_Pinx(x)                                                           \
    ((x[1]) == '0'   ? GPIO_Pin_0                                              \
     : (x[2]) == '5' ? GPIO_Pin_15                                             \
     : (x[2]) == '4' ? GPIO_Pin_14                                             \
     : (x[2]) == '3' ? GPIO_Pin_13                                             \
     : (x[2]) == '2' ? GPIO_Pin_12                                             \
     : (x[2]) == '1' ? GPIO_Pin_11                                             \
     : (x[2]) == '0' ? GPIO_Pin_10                                             \
     : (x[1]) == '9' ? GPIO_Pin_9                                              \
     : (x[1]) == '8' ? GPIO_Pin_8                                              \
     : (x[1]) == '7' ? GPIO_Pin_7                                              \
     : (x[1]) == '6' ? GPIO_Pin_6                                              \
     : (x[1]) == '5' ? GPIO_Pin_5                                              \
     : (x[1]) == '4' ? GPIO_Pin_4                                              \
     : (x[1]) == '3' ? GPIO_Pin_3                                              \
     : (x[1]) == '2' ? GPIO_Pin_2                                              \
     : (x[1]) == '1' ? GPIO_Pin_1                                              \
                     : NULL)

typedef struct {
    char GPIOxPiny[32];
    GPIOMode_TypeDef Mode;

    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
} GPIO;

void GPIO_Init_(GPIO *gpio);

#endif