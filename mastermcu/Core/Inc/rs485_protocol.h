#ifndef __RS485_PROTOCOL_H
#define __RS485_PROTOCOL_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// RS485控制引脚
#define RS485_DE_GPIO_Port  GPIOB
#define RS485_DE_Pin        GPIO_PIN_0

// 通信参数
#define RS485_BAUDRATE      115200
#define RS485_TIMEOUT       200     // 200ms超时
#define RS485_RETRY_MAX     3       // 最大重传次数

// 帧格式定义
#define FRAME_START         0xAA
#define FRAME_END           0x55
#define MAX_DATA_LENGTH     32

// 命令类型
typedef enum {
    CMD_SYNC = 0x00,            // 状态同步
    CMD_DOOR_OPEN = 0x01,
    CMD_DOOR_CLOSE = 0x02,
    CMD_DOOR_STATUS = 0x03,
    CMD_SENSOR_STATUS = 0x04,
    CMD_KEYPAD_DATA = 0x05,
    CMD_HEARTBEAT = 0x06,
    CMD_ACK = 0x07,
    CMD_NACK = 0x08,
    CMD_ERROR = 0x09
} CommandType_t;

// 发送状态机
typedef enum {
    TX_STATE_IDLE = 0,
    TX_STATE_SENDING,
    TX_STATE_WAIT_ACK,
    TX_STATE_COMPLETE,
    TX_STATE_ERROR
} TxState_t;

// 接收状态机
typedef enum {
    RX_STATE_IDLE = 0,
    RX_STATE_RECEIVING,
    RX_STATE_PARSE,
    RX_STATE_LENGTH,
    RX_STATE_CMD,
    RX_STATE_DATA,
    RX_STATE_CRC_LOW,
    RX_STATE_CRC_HIGH,
    RX_STATE_END
} RxState_t;

// 数据帧结构
typedef struct {
    uint8_t start;              // 起始字节
    uint8_t length;             // 数据长度
    uint8_t cmd;                // 命令类型
    uint8_t data[MAX_DATA_LENGTH];  // 数据
    uint16_t crc;               // CRC16校验
    uint8_t end;                // 结束字节
} RS485_Frame_t;

// 发送缓冲区
typedef struct {
    RS485_Frame_t frame;
    TxState_t state;
    uint8_t retry_count;
    uint32_t last_tx_time;
    bool ack_received;
} TxBuffer_t;

// 接收缓冲区
typedef struct {
    RS485_Frame_t frame;
    RxState_t state;
    uint8_t data_index;
    uint32_t last_rx_time;
    uint16_t crc_received;
} RxBuffer_t;

// RS485控制结构体
typedef struct {
    TxBuffer_t tx_buffer;
    RxBuffer_t rx_buffer;
    uint32_t last_sync_time;
    bool sync_pending;
    uint16_t crc_table[256];    // CRC查找表
} RS485_Control_t;

// 全局RS485控制实例
extern RS485_Control_t g_rs485;

// 初始化和处理函数
void RS485_Init(void);
void RS485_Handler(void);

// 发送函数
bool RS485_SendCommand(CommandType_t cmd, uint8_t* data, uint8_t length);
bool RS485_SendSync(void);
bool RS485_SendDoorCommand(uint8_t door_cmd);
bool RS485_RequestSensorStatus(void);
bool RS485_SendHeartbeat(void);

// CRC计算
void RS485_InitCRC(void);
uint16_t RS485_CalculateCRC(uint8_t* data, uint8_t length);

// 内部函数
void RS485_SetMode(bool transmit);
void RS485_ProcessTx(void);
void RS485_ProcessRx(void);
void RS485_ProcessRxFrame(RS485_Frame_t* frame);
void RS485_UART_RxCallback(uint8_t data);

// 状态查询
bool RS485_IsConnected(void);
uint32_t RS485_GetLastRxTime(void);
bool RS485_IsSyncDelayed(void);

#endif /* __RS485_PROTOCOL_H */