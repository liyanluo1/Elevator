#ifndef __RS485_CONFIG_H__
#define __RS485_CONFIG_H__

/* RS485 Hardware Configuration */
#define RS485_UART_INSTANCE     UART5
#define RS485_BAUDRATE          115200
#define RS485_WORDLENGTH        UART_WORDLENGTH_8B
#define RS485_STOPBITS          UART_STOPBITS_1
#define RS485_PARITY            UART_PARITY_NONE
#define RS485_MODE              UART_MODE_TX_RX
#define RS485_HWFLOWCTL         UART_HWCONTROL_NONE
#define RS485_OVERSAMPLING      UART_OVERSAMPLING_16

/* RS485 Pin Configuration */
/* TX: PC12, RX: PD2 */
#define RS485_TX_PIN            GPIO_PIN_12
#define RS485_TX_GPIO_PORT      GPIOC
#define RS485_RX_PIN            GPIO_PIN_2
#define RS485_RX_GPIO_PORT      GPIOD

/* RS485 DMA Configuration */
#define RS485_USE_DMA           1
#define RS485_DMA_TX_STREAM     DMA1_Stream7
#define RS485_DMA_RX_STREAM     DMA1_Stream0
#define RS485_DMA_TX_CHANNEL    DMA_CHANNEL_4
#define RS485_DMA_RX_CHANNEL    DMA_CHANNEL_4

/* RS485 uses auto-direction control chip - no manual control needed */

/* RS485 Buffer Sizes */
#define RS485_RX_BUFFER_SIZE    256
#define RS485_TX_BUFFER_SIZE    256

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