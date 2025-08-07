#ifndef __RS485_CORE_H
#define __RS485_CORE_H

#include "rs485_base.h"

// RS485核心控制结构体
typedef struct {
    // 配置
    RS485_Config_t config;
    
    // 发送管理
    RS485_TxState_t tx_state;
    RS485_Frame_t tx_frame;
    uint8_t tx_buffer[RS485_MAX_FRAME_SIZE];
    uint16_t tx_length;
    uint32_t tx_timestamp;
    uint8_t retry_count;
    bool wait_ack;
    
    // 接收管理
    RS485_RxState_t rx_state;
    RS485_Frame_t rx_frame;
    uint8_t rx_data_index;
    uint32_t rx_timestamp;
    uint16_t rx_crc_temp;
    
    // CRC表
    uint16_t crc_table[256];
    bool crc_table_initialized;
    
    // 连接状态
    bool is_connected;
    uint32_t last_heartbeat_tx;
    uint32_t last_heartbeat_rx;
    
    // 统计信息
    RS485_Stats_t stats;
    
    // 回调函数
    RS485_FrameCallback_t on_frame_received;
    RS485_ErrorCallback_t on_error;
    RS485_EventCallback_t on_event;
    
    // UART接收缓冲
    uint8_t uart_rx_byte;
    
} RS485_Core_t;

// 初始化函数
RS485_Error_t RS485_Core_Init(RS485_Core_t* core, RS485_Config_t* config);
void RS485_Core_DeInit(RS485_Core_t* core);

// 主处理函数
void RS485_Core_Handler(RS485_Core_t* core);

// 发送函数
RS485_Error_t RS485_Core_SendFrame(RS485_Core_t* core, uint8_t cmd, 
                                   uint8_t* data, uint8_t length, bool need_ack);
RS485_Error_t RS485_Core_SendRawFrame(RS485_Core_t* core, RS485_Frame_t* frame);

// 接收处理
void RS485_Core_ProcessRxByte(RS485_Core_t* core, uint8_t byte);
void RS485_Core_ProcessRxFrame(RS485_Core_t* core);

// 控制函数
void RS485_Core_SetMode(RS485_Core_t* core, bool transmit);
void RS485_Core_Reset(RS485_Core_t* core);
void RS485_Core_AbortTx(RS485_Core_t* core);

// 状态查询
bool RS485_Core_IsConnected(RS485_Core_t* core);
bool RS485_Core_IsBusy(RS485_Core_t* core);
RS485_TxState_t RS485_Core_GetTxState(RS485_Core_t* core);
RS485_RxState_t RS485_Core_GetRxState(RS485_Core_t* core);

// 统计信息
RS485_Stats_t* RS485_Core_GetStats(RS485_Core_t* core);
void RS485_Core_ResetStats(RS485_Core_t* core);

// CRC功能
void RS485_Core_InitCRC(RS485_Core_t* core);
uint16_t RS485_Core_CalculateCRC(RS485_Core_t* core, uint8_t* data, uint8_t length);

// 内部函数
void RS485_Core_ProcessTxStateMachine(RS485_Core_t* core);
void RS485_Core_ProcessRxStateMachine(RS485_Core_t* core);
void RS485_Core_CheckHeartbeat(RS485_Core_t* core);
void RS485_Core_TransmitFrame(RS485_Core_t* core);

#endif /* __RS485_CORE_H */