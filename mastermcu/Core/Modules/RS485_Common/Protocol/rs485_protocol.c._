#include "rs485_protocol.h"
#include <string.h>

// 初始化协议
void RS485_Protocol_Init(RS485_Protocol_t* protocol, RS485_ProtocolConfig_t* config) {
    if (protocol == NULL || config == NULL) {
        return;
    }
    
    // 清零结构体
    memset(protocol, 0, sizeof(RS485_Protocol_t));
    
    // 复制配置
    memcpy(&protocol->config, config, sizeof(RS485_ProtocolConfig_t));
    
    // 设置默认值
    if (protocol->config.sync_interval == 0) {
        protocol->config.sync_interval = 200; // 默认200ms
    }
}

// 处理接收到的帧
void RS485_Protocol_ProcessFrame(RS485_Protocol_t* protocol, RS485_Frame_t* frame) {
    if (protocol == NULL || frame == NULL) {
        return;
    }
    
    // 解析命令类型（考虑偏移）
    RS485_CommandType_t cmd_type = RS485_Protocol_ParseCommand(protocol, frame->cmd);
    
    // 根据命令类型分发处理
    switch (cmd_type) {
        case RS485_CMD_SYNC:
            if (protocol->on_sync) {
                protocol->on_sync(frame->data, frame->length);
            }
            break;
            
        case RS485_CMD_HEARTBEAT:
            if (protocol->on_heartbeat && frame->length >= 4) {
                uint32_t timestamp = RS485_Protocol_UnpackUint32(frame->data);
                protocol->on_heartbeat(timestamp);
            }
            break;
            
        case RS485_CMD_DOOR_OPEN:
        case RS485_CMD_DOOR_CLOSE:
        case RS485_CMD_DOOR_STOP:
        case RS485_CMD_DOOR_STATUS:
            if (protocol->on_door_cmd) {
                protocol->on_door_cmd(cmd_type, frame->data, frame->length);
            }
            break;
            
        case RS485_CMD_SENSOR_STATUS:
        case RS485_CMD_SENSOR_DATA:
        case RS485_CMD_SENSOR_TRIGGERED:
            if (protocol->on_sensor_data) {
                protocol->on_sensor_data(frame->data, frame->length);
            }
            break;
            
        case RS485_CMD_KEYPAD_DATA:
            if (protocol->on_keypad_data && frame->length >= 1) {
                protocol->on_keypad_data(frame->data[0]);
            }
            break;
            
        case RS485_CMD_ERROR:
            if (protocol->on_error && frame->length >= 2) {
                uint16_t error_code = RS485_Protocol_UnpackUint16(frame->data);
                protocol->on_error(error_code);
            }
            break;
            
        default:
            // 自定义命令
            if (cmd_type >= RS485_CMD_CUSTOM_BASE && protocol->on_custom) {
                protocol->on_custom(frame->cmd, frame->data, frame->length);
            }
            break;
    }
}

// 构建同步帧
RS485_Error_t RS485_Protocol_BuildSyncFrame(RS485_Frame_t* frame, uint32_t fields, 
                                           uint8_t* sync_data, uint8_t* length) {
    if (frame == NULL || sync_data == NULL || length == NULL) {
        return RS485_ERROR_UNKNOWN;
    }
    
    frame->cmd = RS485_CMD_SYNC;
    frame->length = *length;
    
    if (*length > 0 && *length <= RS485_MAX_DATA_LENGTH) {
        memcpy(frame->data, sync_data, *length);
        return RS485_OK;
    }
    
    return RS485_ERROR_FRAME;
}

// 构建门控制帧
RS485_Error_t RS485_Protocol_BuildDoorFrame(RS485_Frame_t* frame, uint8_t door_cmd, uint8_t* params) {
    if (frame == NULL) {
        return RS485_ERROR_UNKNOWN;
    }
    
    frame->cmd = door_cmd;
    
    if (params != NULL) {
        frame->data[0] = params[0];
        frame->length = 1;
    } else {
        frame->length = 0;
    }
    
    return RS485_OK;
}

// 构建传感器数据帧
RS485_Error_t RS485_Protocol_BuildSensorFrame(RS485_Frame_t* frame, uint8_t floor, int16_t offset) {
    if (frame == NULL) {
        return RS485_ERROR_UNKNOWN;
    }
    
    frame->cmd = RS485_CMD_SENSOR_DATA;
    frame->data[0] = floor;
    RS485_Protocol_PackUint16(&frame->data[1], (uint16_t)offset);
    frame->length = 3;
    
    return RS485_OK;
}

// 构建键盘数据帧
RS485_Error_t RS485_Protocol_BuildKeypadFrame(RS485_Frame_t* frame, uint8_t floor) {
    if (frame == NULL) {
        return RS485_ERROR_UNKNOWN;
    }
    
    frame->cmd = RS485_CMD_KEYPAD_DATA;
    frame->data[0] = floor;
    frame->length = 1;
    
    return RS485_OK;
}

// 构建心跳帧
RS485_Error_t RS485_Protocol_BuildHeartbeatFrame(RS485_Frame_t* frame, uint32_t timestamp) {
    if (frame == NULL) {
        return RS485_ERROR_UNKNOWN;
    }
    
    frame->cmd = RS485_CMD_HEARTBEAT;
    RS485_Protocol_PackUint32(frame->data, timestamp);
    frame->length = 4;
    
    return RS485_OK;
}

// 构建ACK帧
RS485_Error_t RS485_Protocol_BuildAckFrame(RS485_Frame_t* frame, uint8_t ack_cmd) {
    if (frame == NULL) {
        return RS485_ERROR_UNKNOWN;
    }
    
    frame->cmd = RS485_CMD_ACK;
    frame->data[0] = ack_cmd;
    frame->length = 1;
    
    return RS485_OK;
}

// 构建NACK帧
RS485_Error_t RS485_Protocol_BuildNackFrame(RS485_Frame_t* frame, uint8_t nack_cmd, uint8_t reason) {
    if (frame == NULL) {
        return RS485_ERROR_UNKNOWN;
    }
    
    frame->cmd = RS485_CMD_NACK;
    frame->data[0] = nack_cmd;
    frame->data[1] = reason;
    frame->length = 2;
    
    return RS485_OK;
}

// 构建错误帧
RS485_Error_t RS485_Protocol_BuildErrorFrame(RS485_Frame_t* frame, uint16_t error_code) {
    if (frame == NULL) {
        return RS485_ERROR_UNKNOWN;
    }
    
    frame->cmd = RS485_CMD_ERROR;
    RS485_Protocol_PackUint16(frame->data, error_code);
    frame->length = 2;
    
    return RS485_OK;
}

// 获取实际命令（应用偏移）
uint8_t RS485_Protocol_GetCommand(RS485_Protocol_t* protocol, RS485_CommandType_t cmd_type) {
    if (protocol == NULL) {
        return cmd_type;
    }
    
    return cmd_type + protocol->config.cmd_offset;
}

// 解析命令（移除偏移）
RS485_CommandType_t RS485_Protocol_ParseCommand(RS485_Protocol_t* protocol, uint8_t raw_cmd) {
    if (protocol == NULL) {
        return raw_cmd;
    }
    
    // 移除偏移量以获得基础命令类型
    if (raw_cmd >= protocol->config.cmd_offset) {
        return (RS485_CommandType_t)(raw_cmd - protocol->config.cmd_offset);
    }
    
    return (RS485_CommandType_t)raw_cmd;
}

// 打包16位数据（小端序）
void RS485_Protocol_PackUint16(uint8_t* buffer, uint16_t value) {
    if (buffer == NULL) return;
    
    buffer[0] = (uint8_t)(value & 0xFF);
    buffer[1] = (uint8_t)((value >> 8) & 0xFF);
}

// 打包32位数据（小端序）
void RS485_Protocol_PackUint32(uint8_t* buffer, uint32_t value) {
    if (buffer == NULL) return;
    
    buffer[0] = (uint8_t)(value & 0xFF);
    buffer[1] = (uint8_t)((value >> 8) & 0xFF);
    buffer[2] = (uint8_t)((value >> 16) & 0xFF);
    buffer[3] = (uint8_t)((value >> 24) & 0xFF);
}

// 解包16位数据（小端序）
uint16_t RS485_Protocol_UnpackUint16(const uint8_t* buffer) {
    if (buffer == NULL) return 0;
    
    return (uint16_t)(buffer[0] | (buffer[1] << 8));
}

// 解包32位数据（小端序）
uint32_t RS485_Protocol_UnpackUint32(const uint8_t* buffer) {
    if (buffer == NULL) return 0;
    
    return (uint32_t)(buffer[0] | 
                     (buffer[1] << 8) | 
                     (buffer[2] << 16) | 
                     (buffer[3] << 24));
}