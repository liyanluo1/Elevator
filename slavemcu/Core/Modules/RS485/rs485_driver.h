#ifndef __RS485_DRIVER_INTERNAL_H__
#define __RS485_DRIVER_INTERNAL_H__

#include <stdint.h>
#include "stm32f1xx_hal.h"  /* STM32F103 */
#include "rs485_config.h"

/* Internal helper functions */
uint16_t rs485_get_dma_rx_count(UART_HandleTypeDef *huart);

#endif /* __RS485_DRIVER_INTERNAL_H__ */