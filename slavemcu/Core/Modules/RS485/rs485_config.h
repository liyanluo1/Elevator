#ifndef __RS485_CONFIG_H__
#define __RS485_CONFIG_H__

/* RS485 Hardware Configuration */
#define RS485_UART_INSTANCE     USART2  /* slavemcu uses USART2 for RS485 */
#define RS485_BAUDRATE          115200
#define RS485_WORDLENGTH        UART_WORDLENGTH_8B
#define RS485_STOPBITS          UART_STOPBITS_1
#define RS485_PARITY            UART_PARITY_NONE
#define RS485_MODE              UART_MODE_TX_RX
#define RS485_HWFLOWCTL         UART_HWCONTROL_NONE
#define RS485_OVERSAMPLING      UART_OVERSAMPLING_16

/* RS485 Pin Configuration */
/* TX: PA2, RX: PA3 (USART2 on STM32F103) */
#define RS485_TX_PIN            GPIO_PIN_2
#define RS485_TX_GPIO_PORT      GPIOA
#define RS485_RX_PIN            GPIO_PIN_3
#define RS485_RX_GPIO_PORT      GPIOA

/* RS485 DMA Configuration */
/* STM32F103 uses DMA1 Channel 6 for USART2_RX, Channel 7 for USART2_TX */
#define RS485_USE_DMA           1
#define RS485_DMA_TX_CHANNEL    DMA1_Channel7
#define RS485_DMA_RX_CHANNEL    DMA1_Channel6

/* RS485 uses auto-direction control chip - no manual control needed */

/* RS485 Buffer Sizes */
#define RS485_RX_BUFFER_SIZE    512
#define RS485_TX_BUFFER_SIZE    512

/* RS485 Timeout Values */
#define RS485_DEFAULT_TIMEOUT   1000  /* ms */
#define RS485_RX_TIMEOUT        100   /* ms */
#define RS485_TX_TIMEOUT        100   /* ms */

/* RS485 Protocol Configuration */
#define RS485_MAX_PACKET_SIZE   128
#define RS485_PACKET_TIMEOUT    50    /* ms */

/* RS485 Debug Options */
#define RS485_DEBUG_ENABLED     0

#endif /* __RS485_CONFIG_H__ */