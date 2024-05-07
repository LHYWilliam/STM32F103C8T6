#ifndef __DMA_H
#define __DMA_H

#include "stm32f10x.h"
#include <stdint.h>

typedef struct {
    uint32_t RCC_AHBPeriph;
    DMA_Channel_TypeDef *DMAy_Channelx;

    uint32_t DMA_PeripheralBaseAddr;
    uint32_t DMA_PeripheralDataSize;
    uint32_t DMA_PeripheralInc;

    uint32_t DMA_MemoryBaseAddr;
    uint32_t DMA_MemoryDataSize;
    uint32_t DMA_MemoryInc;

    uint32_t DMA_BufferSize;
    uint32_t DMA_DIR;
    uint32_t DMA_Mode;
    uint32_t DMA_Priority;
    uint32_t DMA_M2M;
} DMA;

void DMA_Init_(DMA *dma);
void DMA_Enable(DMA *dma);
void DMA_Disable(DMA *dma);

#endif