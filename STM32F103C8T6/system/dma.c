#include "stm32f10x.h"
#include "stm32f10x_dma.h"

#include <stdlib.h>

#include "dma.h"

void DMA_Init_(DMA *dma) {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMAx(dma->DMAx), ENABLE);

    DMA_InitTypeDef DMA_InitStruct = {
        .DMA_PeripheralBaseAddr = dma->sourceAddr,
        .DMA_PeripheralDataSize = DMA_PeripheralDataSize(dma->DataSize),
        .DMA_PeripheralInc = dma->sourceInc ? DMA_PeripheralInc_Enable
                                            : DMA_PeripheralInc_Disable,

        .DMA_MemoryBaseAddr = dma->targetAddr,
        .DMA_MemoryDataSize = DMA_PeripheralDataSize(dma->DataSize),
        .DMA_MemoryInc =
            dma->targetInc ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable,

        .DMA_DIR = DMA_DIR_PeripheralSRC,
        .DMA_BufferSize = dma->BufferSize,
        .DMA_Priority = DMA_Priority_Medium,
        .DMA_M2M = dma->M2M ? DMA_M2M_Enable : DMA_M2M_Disable,
        .DMA_Mode = dma->Circular ? DMA_Mode_Circular : DMA_Mode_Normal,
    };

    dma->DMAy_Channelx = DMAy_Channelx(dma->DMAx, dma->channel);

    DMA_Init(dma->DMAy_Channelx, &DMA_InitStruct);

    DMA_Cmd(dma->DMAy_Channelx, ENABLE);
}
void DMA_Enable(DMA *dma) { DMA_Cmd(dma->DMAy_Channelx, ENABLE); }

void DMA_Disable(DMA *dma) { DMA_Cmd(dma->DMAy_Channelx, DISABLE); }