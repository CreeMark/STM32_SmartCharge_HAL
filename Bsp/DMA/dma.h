#ifndef _DMA_H_
#define _DMA_H_
#include "sys.h"

#ifdef USE_STDPERIPH_DRIVER
    #include "stm32f10x_dma.h"  // 标准库DMA头文件
    
    void MYDMA_Config(DMA_Channel_TypeDef *DMA_CHx, uint32_t cpar, uint32_t cmar, uint16_t cndtr); // 配置DMA1_CHx
    void MYDMA_Config1(DMA_Channel_TypeDef *DMA_CHx, uint32_t cpar, uint32_t cmar, uint16_t cndtr);
    void MYDMA_Enable(DMA_Channel_TypeDef *DMA_CHx); // 使能DMA1_CHx
#endif

#ifdef USE_HAL_DRIVER
    // HAL库版本DMA功能尚未实现，暂时为空
    // 后续可添加HAL库DMA相关功能
#endif

#endif







