#ifndef __RS485_PROTOCOL_H
#define __RS485_PROTOCOL_H

#include "rs485_base.h"

// 通用命令类型基类
typedef enum {
    // 基础命令 (0x00-0x0F)
    RS485_CMD_SYNC = 0x00,
    RS485_CMD_HEARTBEAT = 0x01,
    RS485_CMD_ACK = 0x02,
    RS485_CMD_NACK = 0x03,
    RS485_CMD_ERROR = 0x04,
    
    // 控制命令 (0x10-0x1F)
    RS485_CMD_DOOR_OPEN = 0x10,
    RS485_CMD_DOOR_CLOSE = 0x11,
    RS485_CMD_DOOR_STOP = 0x12,
    RS485_CMD_DOOR_STATUS = 0x13,
    
    // 传感器命令 (0x20-0x2F)
    RS485_CMD_SENSOR_STATUS = 0x20,
    RS485_CMD_SENSOR_DATA = 0x21,
    RS485_CMD_SENSOR_TRIGGERED = 0x22,
    
    // 键盘命令 (0x30-0x3F)
    RS485_CMD_KEYPAD_DATA = 0x30,
    RS485_CMD_KEYPAD_ACK = 0x31,
    
    // 扩展命令 (0x40+)
    RS485_CMD_CUSTOM_BASE = 0x40
} RS485_CommandType_t;

// 同步数据字段
typedef enum {
    SYNC_FIELD_TARGET_FLOOR = 0x01,
    SYNC_FIELD_CURRENT_FLOOR = 0x02,
    SYNC_FIELD_DIRECTION = 0x04,
    SYNC_FIELD_DOOR_STATE = 0x08,
    SYNC_FIELD_ERROR_CODE = 0x10,
    SYNC_FIELD_POSITION = 0x20,
    SYNC_FIELD_STATE = 0x40,
    SYNC_FIELD_ALL = 0xFF
} RS485_SyncField_t;

// 协议配置结构
typedef struct {
    // 命令映射表
    uint8_t cmd_offset;     // 命令偏移量（用于区分主从）
    
    // 协议参数
    bool use_ack;           // 是否使用ACK确认
    bool use_heartbeat;     // 是否使用心跳
    uint32_t sync_interval; // 同步间隔
    
    // 角色定义
    bool is_master;         // 是否为主机
} RS485_ProtocolConfig_t;

// 协议处理器结构
typedef struct {
    // 配置
    RS485_ProtocolConfig_t config;
    
    // 命令处理回调
    void (*on_sync)(uint8_t* data, uint8_t length);
    void (*on_door_cmd)(uint8_t cmd, uint8_t* data, uint8_t length);
    void (*on_sensor_data)(uint8_t* data, uint8_t length);
    void (*on_keypad_data)(uint8_t floor);
    void (*on_heartbeat)(uint32_t timestamp);
    void (*on_error)(uint16_t error_code);
    void (*on_custom)(uint8_t cmd, uint8_t* data, uint8_t length);
} RS485_Protocol_t;

// 初始化协议
void RS485_Protocol_Init(RS485_Protocol_t* protocol, RS485_ProtocolConfig_t* config);

// 处理接收到的帧
void RS485_Protocol_ProcessFrame(RS485_Protocol_t* protocol, RS485_Frame_t* frame);

// 构建各种命令帧
RS485_Error_t RS485_Protocol_BuildSyncFrame(RS485_Frame_t* frame, uint32_t fields, uint8_t* sync_data, uint8_t* length);
RS485_Error_t RS485_Protocol_BuildDoorFrame(RS485_Frame_t* frame, uint8_t door_cmd, uint8_t* params);
RS485_Error_t RS485_Protocol_BuildSensorFrame(RS485_Frame_t* frame, uint8_t floor, int16_t offset);
RS485_Error_t RS485_Protocol_BuildKeypadFrame(RS485_Frame_t* frame, uint8_t floor);
RS485_Error_t RS485_Protocol_BuildHeartbeatFrame(RS485_Frame_t* frame, uint32_t timestamp);
RS485_Error_t RS485_Protocol_BuildAckFrame(RS485_Frame_t* frame, uint8_t ack_cmd);
RS485_Error_t RS485_Protocol_BuildNackFrame(RS485_Frame_t* frame, uint8_t nack_cmd, uint8_t reason);
RS485_Error_t RS485_Protocol_BuildErrorFrame(RS485_Frame_t* frame, uint16_t error_code);

// 命令转换（考虑偏移）
uint8_t RS485_Protocol_GetCommand(RS485_Protocol_t* protocol, RS485_CommandType_t cmd_type);
RS485_CommandType_t RS485_Protocol_ParseCommand(RS485_Protocol_t* protocol, uint8_t raw_cmd);

// 数据打包/解包辅助函数
void RS485_Protocol_PackUint16(uint8_t* buffer, uint16_t value);
void RS485_Protocol_PackUint32(uint8_t* buffer, uint32_t value);
uint16_t RS485_Protocol_UnpackUint16(const uint8_t* buffer);
uint32_t RS485_Protocol_UnpackUint32(const uint8_t* buffer);

#endif /* __RS485_PROTOCOL_H */