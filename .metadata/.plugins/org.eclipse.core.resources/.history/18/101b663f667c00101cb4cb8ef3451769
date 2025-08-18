#include "rs485.h"
#include <string.h>

/* Global RS485 Instance */
rs485_driver_t rs485_driver;

/* Static Buffers */
static uint8_t rs485_dma_rx_buffer[RS485_DMA_RX_BUFFER_SIZE];
static uint8_t rs485_rx_ring_buffer[RS485_RX_BUFFER_SIZE];
static uint8_t rs485_tx_ring_buffer[RS485_TX_BUFFER_SIZE];

/* Private function prototypes */
static void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t size);
static uint16_t ring_buffer_write(ring_buffer_t *rb, const uint8_t *data, uint16_t length);
static uint16_t ring_buffer_read(ring_buffer_t *rb, uint8_t *data, uint16_t length);
static uint16_t ring_buffer_available(ring_buffer_t *rb);
static void ring_buffer_clear(ring_buffer_t *rb);
static void rs485_process_dma_rx(void);

/**
 * @brief Initialize RS485 driver
 */
rs485_status_t rs485_init(void)
{
    /* Clear driver structure */
    memset(&rs485_driver, 0, sizeof(rs485_driver));
    
    /* Set buffer pointers */
    rs485_driver.dma_rx_buffer = rs485_dma_rx_buffer;
    rs485_driver.rx_buffer = rs485_rx_ring_buffer;
    rs485_driver.tx_buffer = rs485_tx_ring_buffer;
    
    /* Initialize ring buffers */
    ring_buffer_init(&rs485_driver.rx_ring, rs485_rx_ring_buffer, RS485_RX_BUFFER_SIZE);
    ring_buffer_init(&rs485_driver.tx_ring, rs485_tx_ring_buffer, RS485_TX_BUFFER_SIZE);
    
    /* Set UART handle */
    rs485_driver.huart = &huart2;  /* slavemcu uses USART2 */
    
    /* Clear DMA buffer */
    memset(rs485_dma_rx_buffer, 0, RS485_DMA_RX_BUFFER_SIZE);
    
    /* Start DMA reception in circular mode */
    HAL_UART_Receive_DMA(rs485_driver.huart, rs485_dma_rx_buffer, RS485_DMA_RX_BUFFER_SIZE);
    
    /* Set DMA to circular mode (STM32F1 style) */
    rs485_driver.huart->hdmarx->Instance->CCR |= DMA_CCR_CIRC;
    
    /* Enable IDLE interrupt */
    __HAL_UART_ENABLE_IT(rs485_driver.huart, UART_IT_IDLE);
    
    /* Reset statistics */
    rs485_reset_stats();
    
    return RS485_OK;
}

/**
 * @brief Deinitialize RS485 driver
 */
rs485_status_t rs485_deinit(void)
{
    /* Stop DMA */
    HAL_UART_DMAStop(rs485_driver.huart);
    
    /* Disable IDLE interrupt */
    __HAL_UART_DISABLE_IT(rs485_driver.huart, UART_IT_IDLE);
    
    return RS485_OK;
}

/**
 * @brief Send packet via RS485 using DMA (non-blocking)
 */
rs485_status_t rs485_send_packet_dma(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;
    
    if (data == NULL || length == 0 || length > RS485_TX_BUFFER_SIZE) {
        return RS485_ERROR;
    }
    
    if (rs485_driver.tx_pending) {
        return RS485_BUSY;
    }
    
    /* Set pending flag */
    rs485_driver.tx_pending = 1;
    
    /* Copy data to TX buffer */
    memcpy(rs485_driver.tx_buffer, data, length);
    
    /* Send data via DMA */
    status = HAL_UART_Transmit_DMA(rs485_driver.huart, rs485_driver.tx_buffer, length);
    
    if (status == HAL_OK) {
        rs485_driver.stats.tx_packets++;
        rs485_driver.stats.tx_bytes += length;
        return RS485_OK;
    } else {
        rs485_driver.stats.tx_errors++;
        rs485_driver.tx_pending = 0;
        return RS485_ERROR;
    }
}

/**
 * @brief Receive packet from RS485
 */
uint16_t rs485_receive_packet(uint8_t *buffer, uint16_t max_length)
{
    if (buffer == NULL || max_length == 0) {
        return 0;
    }
    
    /* Process any pending DMA data */
    rs485_process_dma_rx();
    
    /* Read from ring buffer */
    uint16_t received = ring_buffer_read(&rs485_driver.rx_ring, buffer, max_length);
    
    if (received > 0) {
        rs485_driver.stats.rx_bytes += received;
    }
    
    return received;
}

/**
 * @brief Get number of bytes available in receive buffer
 */
uint16_t rs485_data_available(void)
{
    /* Process any pending DMA data */
    rs485_process_dma_rx();
    
    return ring_buffer_available(&rs485_driver.rx_ring);
}

/**
 * @brief Clear receive buffer
 */
void rs485_clear_rx_buffer(void)
{
    ring_buffer_clear(&rs485_driver.rx_ring);
}

/**
 * @brief Handle IDLE interrupt for RS485
 */
void rs485_idle_interrupt_handler(void)
{
    /* Clear IDLE flag */
    __HAL_UART_CLEAR_IDLEFLAG(rs485_driver.huart);
    
    /* Process received data */
    rs485_process_dma_rx();
    
    rs485_driver.stats.rx_packets++;
    
    /* Call packet callback if set */
    if (rs485_driver.packet_received_callback != NULL) {
        uint16_t available = ring_buffer_available(&rs485_driver.rx_ring);
        if (available > 0) {
            uint8_t temp_buffer[RS485_RX_BUFFER_SIZE];
            uint16_t length = ring_buffer_read(&rs485_driver.rx_ring, temp_buffer, available);
            rs485_driver.packet_received_callback(temp_buffer, length);
        }
    }
}

/**
 * @brief TX complete callback
 */
void rs485_tx_complete_callback(void)
{
    /* Clear pending flag */
    rs485_driver.tx_pending = 0;
}

/**
 * @brief RX callback from UART layer
 */
void rs485_rx_callback(uint8_t *data, uint16_t length)
{
    /* Call user callback if set */
    if (rs485_driver.packet_received_callback != NULL) {
        rs485_driver.packet_received_callback(data, length);
    }
}

/**
 * @brief Set packet received callback
 */
void rs485_set_packet_callback(void (*callback)(uint8_t *, uint16_t))
{
    rs485_driver.packet_received_callback = callback;
}

/**
 * @brief Get RS485 statistics
 */
void rs485_get_stats(rs485_stats_t *stats)
{
    if (stats != NULL) {
        memcpy(stats, &rs485_driver.stats, sizeof(rs485_stats_t));
    }
}

/**
 * @brief Reset RS485 statistics
 */
void rs485_reset_stats(void)
{
    memset(&rs485_driver.stats, 0, sizeof(rs485_stats_t));
}

/**
 * @brief Print RS485 statistics
 */
void rs485_print_stats(void)
{
    /* Can be implemented with printf if available */
}

/* Private functions - Ring Buffer Implementation */

/**
 * @brief Initialize ring buffer
 */
static void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t size)
{
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
}

/**
 * @brief Write data to ring buffer
 */
static uint16_t ring_buffer_write(ring_buffer_t *rb, const uint8_t *data, uint16_t length)
{
    uint16_t written = 0;
    
    while (written < length) {
        uint16_t next_head = (rb->head + 1) % rb->size;
        
        /* Check if buffer is full */
        if (next_head == rb->tail) {
            break;  /* Buffer full */
        }
        
        rb->buffer[rb->head] = data[written];
        rb->head = next_head;
        written++;
    }
    
    return written;
}

/**
 * @brief Read data from ring buffer
 */
static uint16_t ring_buffer_read(ring_buffer_t *rb, uint8_t *data, uint16_t length)
{
    uint16_t read = 0;
    
    while (read < length && rb->tail != rb->head) {
        data[read] = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % rb->size;
        read++;
    }
    
    return read;
}

/**
 * @brief Get available data in ring buffer
 */
static uint16_t ring_buffer_available(ring_buffer_t *rb)
{
    if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    } else {
        return rb->size - rb->tail + rb->head;
    }
}

/**
 * @brief Clear ring buffer
 */
static void ring_buffer_clear(ring_buffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

/**
 * @brief Process DMA received data
 */
static void rs485_process_dma_rx(void)
{
    /* Get current DMA position */
    uint16_t dma_pos = RS485_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(rs485_driver.huart->hdmarx);
    
    /* Calculate received bytes */
    uint16_t length;
    if (dma_pos >= rs485_driver.last_dma_pos) {
        length = dma_pos - rs485_driver.last_dma_pos;
    } else {
        /* DMA wrapped around */
        length = RS485_DMA_RX_BUFFER_SIZE - rs485_driver.last_dma_pos + dma_pos;
    }
    
    if (length > 0) {
        /* Copy data to ring buffer */
        if (rs485_driver.last_dma_pos + length <= RS485_DMA_RX_BUFFER_SIZE) {
            /* Simple copy */
            ring_buffer_write(&rs485_driver.rx_ring, 
                            &rs485_dma_rx_buffer[rs485_driver.last_dma_pos], 
                            length);
        } else {
            /* Wrapped copy */
            uint16_t first_part = RS485_DMA_RX_BUFFER_SIZE - rs485_driver.last_dma_pos;
            ring_buffer_write(&rs485_driver.rx_ring, 
                            &rs485_dma_rx_buffer[rs485_driver.last_dma_pos], 
                            first_part);
            ring_buffer_write(&rs485_driver.rx_ring, 
                            &rs485_dma_rx_buffer[0], 
                            length - first_part);
        }
        
        /* Update last position */
        rs485_driver.last_dma_pos = dma_pos;
    }
}