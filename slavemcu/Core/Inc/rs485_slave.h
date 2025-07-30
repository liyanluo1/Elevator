#ifndef __RS485_SLAVE_H
#define __RS485_SLAVE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "local_blackboard.h"

// RS485 GPIO配置
#define RS485_DE_GPIO_Port  GPIOB
#define RS485_DE_Pin        GPIO_PIN_0

// 通信参数
#define RS485_BAUDRATE      115200
#define RS485_TIMEOUT       200     // 200ms超时
#define RS485_RETRY_MAX     3       // 最大重试次数

// 帧格式定义
#define FRAME_START         0xAA
#define FRAME_END           0x55
#define MAX_DATA_LENGTH     32
#define CRC_POLY            0x1021  // CRC-16多项式

// 命令类型枚举
typedef enum {
    CMD_SYNC_REQUEST = 0x10,        // Master请求同步
    CMD_SYNC_DATA = 0x11,           // Slave发送同步数据
    CMD_DOOR_COMMAND = 0x20,        // 门控制命令
    CMD_SENSOR_DATA = 0x30,         // 传感器数据
    CMD_KEYBOARD_DATA = 0x40,       // 键盘数据
    CMD_HEARTBEAT = 0x50,           // 心跳
    CMD_ACK = 0x60,                 // 确认
    CMD_NACK = 0x61,                // 否认
    CMD_ERROR = 0x70                // 错误报告
} RS485_CommandType_t;

// 同步数据子命令
typedef enum {
    SYNC_TARGET_FLOOR = 0x01,
    SYNC_DOOR_STATE = 0x02,
    SYNC_CURRENT_FLOOR = 0x03,
    SYNC_ERROR_CODE = 0x04,
    SYNC_POSITION_OFFSET = 0x05,
    SYNC_ALL = 0xFF
} SyncSubCommand_t;

// 数据帧结构
typedef struct {
    uint8_t start;                      // 起始标志
    uint8_t length;                     // 数据长度
    uint8_t cmd;                        // 命令类型
    uint8_t data[MAX_DATA_LENGTH];      // 数据负载
    uint16_t crc;                       // CRC校验
    uint8_t end;                        // 结束标志
} RS485_Frame_t;

// 发送状态枚举
typedef enum {
    TX_STATE_IDLE = 0,
    TX_STATE_SENDING,
    TX_STATE_WAIT_ACK,
    TX_STATE_COMPLETE,
    TX_STATE_ERROR
} TxState_t;

// 接收状态枚举
typedef enum {
    RX_STATE_IDLE = 0,
    RX_STATE_LENGTH,
    RX_STATE_CMD,
    RX_STATE_DATA,
    RX_STATE_CRC_HIGH,
    RX_STATE_CRC_LOW,
    RX_STATE_END
} RxState_t;

// RS485从机控制结构体
typedef struct {
    // 发送相关
    TxState_t tx_state;
    RS485_Frame_t tx_frame;
    uint8_t tx_buffer[64];
    uint16_t tx_length;
    uint32_t last_tx_time;
    uint8_t retry_count;
    
    // 接收相关
    RxState_t rx_state;
    RS485_Frame_t rx_frame;
    uint8_t rx_buffer[64];
    uint8_t rx_index;
    uint8_t rx_data_count;
    uint32_t last_rx_time;
    
    // CRC表
    uint16_t crc_table[256];
    
    // 连接状态
    bool master_connected;
    uint32_t last_heartbeat_time;
    uint16_t comm_error_count;
    
    // 同步管理
    bool sync_pending;
    uint32_t sync_fields_pending;
    uint32_t last_sync_time;
    
} RS485_Slave_t;

// 全局RS485从机实例
extern RS485_Slave_t g_rs485_slave;

// 初始化和主处理函数
void RS485_Slave_Init(void);
void RS485_Slave_Handler(void);

// 发送函数
bool RS485_Slave_SendSync(uint32_t fields);
bool RS485_Slave_SendSensorData(uint8_t floor, int offset);
bool RS485_Slave_SendKeyboardData(uint8_t floor);
bool RS485_Slave_SendDoorStatus(DoorState_t state);
bool RS485_Slave_SendHeartbeat(void);
bool RS485_Slave_SendError(int error_code);
bool RS485_Slave_SendACK(uint8_t cmd);
bool RS485_Slave_SendNACK(uint8_t cmd, uint8_t reason);

// 内部函数
void RS485_Slave_ProcessRxFrame(RS485_Frame_t* frame);
void RS485_Slave_ProcessCommand(uint8_t cmd, uint8_t* data, uint8_t length);
bool RS485_Slave_BuildFrame(RS485_CommandType_t cmd, uint8_t* data, uint8_t length);
bool RS485_Slave_TransmitFrame(void);
void RS485_Slave_SetMode(bool transmit);
uint16_t RS485_Slave_CalculateCRC(uint8_t* data, uint8_t length);
void RS485_Slave_InitCRCTable(void);

// UART回调
void RS485_Slave_UART_RxCallback(uint8_t data);
void RS485_Slave_UART_TxCompleteCallback(void);
void RS485_Slave_UART_ErrorCallback(void);

// 状态查询
bool RS485_Slave_IsConnected(void);
uint32_t RS485_Slave_GetLastRxTime(void);
uint16_t RS485_Slave_GetErrorCount(void);

// 同步检测和处理
void RS485_Slave_CheckSync(void);
void RS485_Slave_UpdateFromBlackboard(void);

#endif /* __RS485_SLAVE_H */