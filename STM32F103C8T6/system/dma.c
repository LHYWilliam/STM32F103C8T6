#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"

#include "dma.h"

void DMA_Init_(DMA *dma) {
    RCC_AHBPeriphClockCmd(dma->RCC_AHBPeriph, ENABLE);

    DMA_InitTypeDef DMA_InitStruct = {
        dma->DMA_PeripheralBaseAddr,
        dma->DMA_MemoryBaseAddr,
        dma->DMA_DIR,
        dma->DMA_BufferSize,
        dma->DMA_PeripheralInc,
        dma->DMA_MemoryInc,
        dma->DMA_PeripheralDataSize,
        dma->DMA_MemoryDataSize,
        dma->DMA_Mode,
        dma->DMA_Priority,
        dma->DMA_M2M,
    };
    DMA_Init(dma->DMAy_Channelx, &DMA_InitStruct);
}
void DMA_Enable(DMA *dma) { DMA_Cmd(dma->DMAy_Channelx, ENABLE); }

void DMA_Disable(DMA *dma) { DMA_Cmd(dma->DMAy_Channelx, DISABLE); }