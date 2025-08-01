#ifndef __RS485_MASTER_H
#define __RS485_MASTER_H

#include "../RS485_Common/Core/rs485_core.h"
#include "../RS485_Common/Protocol/rs485_protocol.h"
#include "blackboard.h"

// Master特定的命令偏移
#define MASTER_CMD_OFFSET  0x00

// Master控制结构
typedef struct {
    // 核心组件
    RS485_Core_t core;
    RS485_Protocol_t protocol;
    
    // 黑板引用
    Blackboard_t* blackboard;
    
    // 同步管理
    uint32_t last_sync_time;
    bool sync_pending;
    
    // Master特定状态
    bool slave_connected;
    uint32_t slave_last_seen;
    
} RS485_Master_t;

// 全局Master实例
extern RS485_Master_t g_rs485_master;

// 初始化函数
void RS485_Master_Init(Blackboard_t* blackboard);
void RS485_Master_Handler(void);

// 高级发送函数
bool RS485_Master_SendSync(void);
bool RS485_Master_SendDoorCommand(uint8_t door_cmd);
bool RS485_Master_SendSensorRequest(void);
bool RS485_Master_SendHeartbeat(void);

// 状态查询
bool RS485_Master_IsSlaveConnected(void);
uint32_t RS485_Master_GetLastRxTime(void);

// 内部回调函数
void RS485_Master_OnFrameReceived(RS485_Frame_t* frame);
void RS485_Master_OnError(RS485_Error_t error);
void RS485_Master_OnEvent(uint8_t event, uint32_t data);

// 协议处理回调
void RS485_Master_OnSync(uint8_t* data, uint8_t length);
void RS485_Master_OnDoorCmd(uint8_t cmd, uint8_t* data, uint8_t length);
void RS485_Master_OnSensorData(uint8_t* data, uint8_t length);
void RS485_Master_OnKeypadData(uint8_t floor);
void RS485_Master_OnHeartbeat(uint32_t timestamp);
void RS485_Master_OnProtocolError(uint16_t error_code);

#endif /* __RS485_MASTER_H */