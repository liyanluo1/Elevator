#ifndef __RS485_BASE_H
#define __RS485_BASE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef STM32F1
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#endif

// 通用帧格式定义
#define RS485_FRAME_START         0xAA
#define RS485_FRAME_END           0x55
#define RS485_MAX_DATA_LENGTH     32
#define RS485_MAX_FRAME_SIZE      64

// 默认通信参数
#define RS485_DEFAULT_BAUDRATE    115200
#define RS485_DEFAULT_TIMEOUT     200     // ms
#define RS485_DEFAULT_RETRY_MAX   3
#define RS485_HEARTBEAT_INTERVAL  100     // ms

// CRC多项式
#define RS485_CRC16_POLY          0x1021  // CRC16-CCITT

// 硬件配置结构体
typedef struct {
    // GPIO配置
    GPIO_TypeDef* de_gpio_port;
    uint16_t de_pin;
    
    // UART配置
    UART_HandleTypeDef* uart_handle;
    uint32_t baudrate;
    
    // 通信参数
    uint32_t timeout_ms;
    uint8_t retry_max;
    uint32_t heartbeat_interval;
} RS485_Config_t;

// 通用数据帧结构
typedef struct {
    uint8_t start;
    uint8_t length;
    uint8_t cmd;
    uint8_t data[RS485_MAX_DATA_LENGTH];
    uint16_t crc;
    uint8_t end;
} RS485_Frame_t;

// 发送状态枚举
typedef enum {
    RS485_TX_IDLE = 0,
    RS485_TX_SENDING,
    RS485_TX_WAIT_ACK,
    RS485_TX_COMPLETE,
    RS485_TX_ERROR
} RS485_TxState_t;

// 接收状态枚举
typedef enum {
    RS485_RX_IDLE = 0,
    RS485_RX_LENGTH,
    RS485_RX_CMD,
    RS485_RX_DATA,
    RS485_RX_CRC_LOW,
    RS485_RX_CRC_HIGH,
    RS485_RX_END
} RS485_RxState_t;

// 错误码定义
typedef enum {
    RS485_OK = 0,
    RS485_ERROR_TIMEOUT,
    RS485_ERROR_CRC,
    RS485_ERROR_FRAME,
    RS485_ERROR_BUSY,
    RS485_ERROR_NACK,
    RS485_ERROR_UNKNOWN
} RS485_Error_t;

// 统计信息
typedef struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
    uint32_t retry_count;
    uint32_t crc_error_count;
    uint32_t timeout_count;
} RS485_Stats_t;

// 回调函数类型定义
typedef void (*RS485_FrameCallback_t)(RS485_Frame_t* frame);
typedef void (*RS485_ErrorCallback_t)(RS485_Error_t error);
typedef void (*RS485_EventCallback_t)(uint8_t event, uint32_t data);

#endif /* __RS485_BASE_H */