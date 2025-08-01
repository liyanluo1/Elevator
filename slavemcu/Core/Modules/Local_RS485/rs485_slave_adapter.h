#ifndef __RS485_SLAVE_ADAPTER_H
#define __RS485_SLAVE_ADAPTER_H

#include "../RS485_Common/Core/rs485_core.h"
#include "../RS485_Common/Protocol/rs485_protocol.h"
#include "local_blackboard.h"

// Slave特定的命令偏移
#define SLAVE_CMD_OFFSET  0x10

// Slave控制结构
typedef struct {
    // 核心组件
    RS485_Core_t core;
    RS485_Protocol_t protocol;
    
    // 黑板引用
    LocalBlackboard_t* blackboard;
    
    // 同步管理
    uint32_t last_sync_time;
    uint32_t sync_fields_pending;
    
    // Slave特定状态
    bool master_connected;
    uint32_t master_last_seen;
    
} RS485_SlaveAdapter_t;

// 全局Slave实例
extern RS485_SlaveAdapter_t g_rs485_slave_adapter;

// 初始化函数
void RS485_SlaveAdapter_Init(LocalBlackboard_t* blackboard);
void RS485_SlaveAdapter_Handler(void);

// 高级发送函数
bool RS485_SlaveAdapter_SendSync(uint32_t fields);
bool RS485_SlaveAdapter_SendSensorData(uint8_t floor, int16_t offset);
bool RS485_SlaveAdapter_SendKeypadData(uint8_t floor);
bool RS485_SlaveAdapter_SendDoorStatus(DoorState_t state);
bool RS485_SlaveAdapter_SendHeartbeat(void);
bool RS485_SlaveAdapter_SendError(int error_code);

// 状态查询
bool RS485_SlaveAdapter_IsMasterConnected(void);
uint32_t RS485_SlaveAdapter_GetLastRxTime(void);

// 同步管理
void RS485_SlaveAdapter_CheckSync(void);
void RS485_SlaveAdapter_UpdateFromBlackboard(void);

// 内部回调函数
void RS485_SlaveAdapter_OnFrameReceived(RS485_Frame_t* frame);
void RS485_SlaveAdapter_OnError(RS485_Error_t error);
void RS485_SlaveAdapter_OnEvent(uint8_t event, uint32_t data);

// 协议处理回调
void RS485_SlaveAdapter_OnSync(uint8_t* data, uint8_t length);
void RS485_SlaveAdapter_OnDoorCmd(uint8_t cmd, uint8_t* data, uint8_t length);
void RS485_SlaveAdapter_OnSensorData(uint8_t* data, uint8_t length);
void RS485_SlaveAdapter_OnKeypadData(uint8_t floor);
void RS485_SlaveAdapter_OnHeartbeat(uint32_t timestamp);
void RS485_SlaveAdapter_OnProtocolError(uint16_t error_code);

#endif /* __RS485_SLAVE_ADAPTER_H */