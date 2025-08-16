#include "rs485_driver.h"
#include "rs485.h"

/* This file is kept for compatibility but most functions are integrated in rs485.c */

/**
 * @brief Get number of bytes received by DMA
 */
uint16_t rs485_get_dma_rx_count(UART_HandleTypeDef *huart)
{
    uint32_t dma_counter;
    
    /* Get DMA counter value */
    dma_counter = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    
    /* Calculate received bytes */
    if (dma_counter <= RS485_DMA_RX_BUFFER_SIZE) {
        return RS485_DMA_RX_BUFFER_SIZE - dma_counter;
    }
    
    return 0;
}