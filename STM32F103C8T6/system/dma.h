#ifndef __DMA_H
#define __DMA_H

#include "stm32f10x.h"

#define RCC_AHBPeriph_DMAx(x)                                                  \
    ((x) == DMA1 ? RCC_AHBPeriph_DMA1 : (x) == DMA2 ? RCC_AHBPeriph_DMA2 : NULL)

#define DMAy_Channelx(x, y)                                                    \
    ((x) == DMA1 && (y) == 1   ? DMA1_Channel1                                 \
     : (x) == DMA1 && (y) == 2 ? DMA1_Channel2                                 \
     : (x) == DMA1 && (y) == 3 ? DMA1_Channel3                                 \
     : (x) == DMA1 && (y) == 4 ? DMA1_Channel4                                 \
     : (x) == DMA1 && (y) == 5 ? DMA1_Channel5                                 \
     : (x) == DMA1 && (y) == 6 ? DMA1_Channel6                                 \
     : (x) == DMA1 && (y) == 7 ? DMA1_Channel7                                 \
     : (x) == DMA2 && (y) == 1 ? DMA2_Channel1                                 \
     : (x) == DMA2 && (y) == 2 ? DMA2_Channel2                                 \
     : (x) == DMA2 && (y) == 3 ? DMA2_Channel3                                 \
     : (x) == DMA2 && (y) == 4 ? DMA2_Channel4                                 \
     : (x) == DMA2 && (y) == 5 ? DMA2_Channel5                                 \
                               : NULL)

#define DMA_PeripheralDataSize(x)                                              \
    ((x) == 8    ? DMA_PeripheralDataSize_Byte                                 \
     : (x) == 16 ? DMA_PeripheralDataSize_HalfWord                             \
     : (x) == 32 ? DMA_PeripheralDataSize_Word                                 \
                 : NULL)

#define DMA_MemoryDataSize(x)                                                  \
    ((x) == 8    ? DMA_MemoryDataSize_Byte                                     \
     : (x) == 16 ? DMA_MemoryDataSize_HalfWord                                 \
     : (x) == 32 ? DMA_MemoryDataSize_Word                                     \
                 : NULL)

typedef struct {
    DMA_TypeDef *DMAx;
    uint8_t channel;

    uint32_t sourceAddr;
    uint8_t sourceInc;

    uint32_t targetAddr;
    uint8_t targetInc;

    uint8_t DataSize;
    uint32_t BufferSize;
    uint32_t Circular;
    uint8_t M2M;

    DMA_Channel_TypeDef *DMAy_Channelx;
} DMA;

void DMA_Init_(DMA *dma);
void DMA_Start(DMA *dma);

#endif