#include "rs485_slave_adapter.h"
#include "usart.h"
#include <string.h>

// 外部UART句柄
extern UART_HandleTypeDef huart2;

// 全局Slave实例
RS485_SlaveAdapter_t g_rs485_slave_adapter;

// 初始化Slave
void RS485_SlaveAdapter_Init(LocalBlackboard_t* blackboard) {
    // 清零结构体
    memset(&g_rs485_slave_adapter, 0, sizeof(RS485_SlaveAdapter_t));
    
    // 保存黑板引用
    g_rs485_slave_adapter.blackboard = blackboard;
    
    // 配置硬件
    RS485_Config_t config = {
        .de_gpio_port = GPIOB,
        .de_pin = GPIO_PIN_0,
        .uart_handle = &huart2,
        .baudrate = 115200,
        .timeout_ms = 200,
        .retry_max = 3,
        .heartbeat_interval = 100
    };
    
    // 初始化核心
    RS485_Core_Init(&g_rs485_slave_adapter.core, &config);
    
    // 设置核心回调
    g_rs485_slave_adapter.core.on_frame_received = RS485_SlaveAdapter_OnFrameReceived;
    g_rs485_slave_adapter.core.on_error = RS485_SlaveAdapter_OnError;
    g_rs485_slave_adapter.core.on_event = RS485_SlaveAdapter_OnEvent;
    
    // 配置协议
    RS485_ProtocolConfig_t protocol_config = {
        .cmd_offset = SLAVE_CMD_OFFSET,
        .use_ack = true,
        .use_heartbeat = true,
        .sync_interval = 200,
        .is_master = false
    };
    
    // 初始化协议
    RS485_Protocol_Init(&g_rs485_slave_adapter.protocol, &protocol_config);
    
    // 设置协议回调
    g_rs485_slave_adapter.protocol.on_sync = RS485_SlaveAdapter_OnSync;
    g_rs485_slave_adapter.protocol.on_door_cmd = RS485_SlaveAdapter_OnDoorCmd;
    g_rs485_slave_adapter.protocol.on_sensor_data = RS485_SlaveAdapter_OnSensorData;
    g_rs485_slave_adapter.protocol.on_keypad_data = RS485_SlaveAdapter_OnKeypadData;
    g_rs485_slave_adapter.protocol.on_heartbeat = RS485_SlaveAdapter_OnHeartbeat;
    g_rs485_slave_adapter.protocol.on_error = RS485_SlaveAdapter_OnProtocolError;
}

// 主处理函数
void RS485_SlaveAdapter_Handler(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 处理核心
    RS485_Core_Handler(&g_rs485_slave_adapter.core);
    
    // 检查连接超时
    if (current_time - g_rs485_slave_adapter.master_last_seen > g_rs485_slave_adapter.core.config.timeout_ms * 2) {
        if (g_rs485_slave_adapter.master_connected) {
            g_rs485_slave_adapter.master_connected = false;
            LocalBlackboard_PushEvent(EVENT_SYNC_TIMEOUT, 0);
        }
    }
    
    // 检查心跳
    if (current_time - g_rs485_slave_adapter.core.last_heartbeat_tx > g_rs485_slave_adapter.core.config.heartbeat_interval) {
        RS485_SlaveAdapter_SendHeartbeat();
    }
    
    // 检查同步
    RS485_SlaveAdapter_CheckSync();
    
    // 更新黑板数据
    RS485_SlaveAdapter_UpdateFromBlackboard();
}

// 发送同步数据
bool RS485_SlaveAdapter_SendSync(uint32_t fields) {
    uint8_t sync_data[16];
    uint8_t index = 0;
    
    // 根据字段标志打包数据
    if (fields & SYNC_FIELD_TARGET_FLOOR) {
        sync_data[index++] = SYNC_FIELD_TARGET_FLOOR;
        sync_data[index++] = g_rs485_slave_adapter.blackboard->target_floor;
    }
    
    if (fields & SYNC_FIELD_DOOR_STATE) {
        sync_data[index++] = SYNC_FIELD_DOOR_STATE;
        sync_data[index++] = (uint8_t)g_rs485_slave_adapter.blackboard->door;
    }
    
    if (fields & SYNC_FIELD_CURRENT_FLOOR) {
        sync_data[index++] = SYNC_FIELD_CURRENT_FLOOR;
        sync_data[index++] = g_rs485_slave_adapter.blackboard->current_floor;
    }
    
    if (fields & SYNC_FIELD_ERROR_CODE) {
        sync_data[index++] = SYNC_FIELD_ERROR_CODE;
        RS485_Protocol_PackUint16(&sync_data[index], g_rs485_slave_adapter.blackboard->error_code);
        index += 2;
    }
    
    if (fields & SYNC_FIELD_POSITION) {
        sync_data[index++] = SYNC_FIELD_POSITION;
        RS485_Protocol_PackUint16(&sync_data[index], g_rs485_slave_adapter.blackboard->position_offset);
        index += 2;
    }
    
    // 构建帧
    RS485_Frame_t frame;
    uint8_t length = index;
    RS485_Protocol_BuildSyncFrame(&frame, fields, sync_data, &length);
    
    // 获取实际命令
    frame.cmd = RS485_Protocol_GetCommand(&g_rs485_slave_adapter.protocol, RS485_CMD_SYNC);
    
    // 发送
    return RS485_Core_SendRawFrame(&g_rs485_slave_adapter.core, &frame) == RS485_OK;
}

// 发送传感器数据
bool RS485_SlaveAdapter_SendSensorData(uint8_t floor, int16_t offset) {
    RS485_Frame_t frame;
    RS485_Protocol_BuildSensorFrame(&frame, floor, offset);
    
    frame.cmd = RS485_Protocol_GetCommand(&g_rs485_slave_adapter.protocol, RS485_CMD_SENSOR_DATA);
    
    return RS485_Core_SendRawFrame(&g_rs485_slave_adapter.core, &frame) == RS485_OK;
}

// 发送键盘数据
bool RS485_SlaveAdapter_SendKeypadData(uint8_t floor) {
    RS485_Frame_t frame;
    RS485_Protocol_BuildKeypadFrame(&frame, floor);
    
    frame.cmd = RS485_Protocol_GetCommand(&g_rs485_slave_adapter.protocol, RS485_CMD_KEYPAD_DATA);
    
    return RS485_Core_SendRawFrame(&g_rs485_slave_adapter.core, &frame) == RS485_OK;
}

// 发送门状态
bool RS485_SlaveAdapter_SendDoorStatus(DoorState_t state) {
    uint8_t data[2];
    data[0] = (uint8_t)state;
    data[1] = g_rs485_slave_adapter.blackboard->servo_position;
    
    RS485_Frame_t frame;
    frame.cmd = RS485_Protocol_GetCommand(&g_rs485_slave_adapter.protocol, RS485_CMD_DOOR_STATUS);
    frame.length = 2;
    memcpy(frame.data, data, 2);
    
    return RS485_Core_SendRawFrame(&g_rs485_slave_adapter.core, &frame) == RS485_OK;
}

// 发送心跳
bool RS485_SlaveAdapter_SendHeartbeat(void) {
    RS485_Frame_t frame;
    uint32_t timestamp = HAL_GetTick();
    RS485_Protocol_BuildHeartbeatFrame(&frame, timestamp);
    
    frame.cmd = RS485_Protocol_GetCommand(&g_rs485_slave_adapter.protocol, RS485_CMD_HEARTBEAT);
    
    g_rs485_slave_adapter.core.last_heartbeat_tx = timestamp;
    
    return RS485_Core_SendRawFrame(&g_rs485_slave_adapter.core, &frame) == RS485_OK;
}

// 发送错误
bool RS485_SlaveAdapter_SendError(int error_code) {
    RS485_Frame_t frame;
    RS485_Protocol_BuildErrorFrame(&frame, (uint16_t)error_code);
    
    frame.cmd = RS485_Protocol_GetCommand(&g_rs485_slave_adapter.protocol, RS485_CMD_ERROR);
    
    return RS485_Core_SendRawFrame(&g_rs485_slave_adapter.core, &frame) == RS485_OK;
}

// 查询连接状态
bool RS485_SlaveAdapter_IsMasterConnected(void) {
    return g_rs485_slave_adapter.master_connected;
}

// 获取最后接收时间
uint32_t RS485_SlaveAdapter_GetLastRxTime(void) {
    return g_rs485_slave_adapter.master_last_seen;
}

// 检查同步
void RS485_SlaveAdapter_CheckSync(void) {
    if (LocalBlackboard_NeedsSync()) {
        uint32_t fields = LocalBlackboard_GetSyncFields();
        if (RS485_SlaveAdapter_SendSync(fields)) {
            LocalBlackboard_ClearSyncFlags();
        }
    }
}

// 从黑板更新数据
void RS485_SlaveAdapter_UpdateFromBlackboard(void) {
    static DoorState_t last_door_state = DOOR_CLOSED;
    static uint8_t last_floor = 1;
    
    // 检查门状态变化
    if (g_rs485_slave_adapter.blackboard->door != last_door_state) {
        RS485_SlaveAdapter_SendDoorStatus(g_rs485_slave_adapter.blackboard->door);
        last_door_state = g_rs485_slave_adapter.blackboard->door;
    }
    
    // 检查楼层变化
    if (g_rs485_slave_adapter.blackboard->current_floor != last_floor) {
        RS485_SlaveAdapter_SendSensorData(g_rs485_slave_adapter.blackboard->current_floor, 
                                         g_rs485_slave_adapter.blackboard->position_offset);
        last_floor = g_rs485_slave_adapter.blackboard->current_floor;
    }
}

// 帧接收回调
void RS485_SlaveAdapter_OnFrameReceived(RS485_Frame_t* frame) {
    // 更新接收时间
    g_rs485_slave_adapter.master_last_seen = HAL_GetTick();
    
    // 协议处理
    RS485_Protocol_ProcessFrame(&g_rs485_slave_adapter.protocol, frame);
}

// 错误回调
void RS485_SlaveAdapter_OnError(RS485_Error_t error) {
    g_rs485_slave_adapter.core.stats.error_count++;
    
    switch (error) {
        case RS485_ERROR_TIMEOUT:
            LocalBlackboard_PushEvent(EVENT_SYNC_TIMEOUT, 0);
            break;
        case RS485_ERROR_CRC:
            // CRC错误已在核心层统计
            break;
        default:
            LocalBlackboard_PushEvent(EVENT_ERROR, 0x100 + error);
            break;
    }
}

// 事件回调
void RS485_SlaveAdapter_OnEvent(uint8_t event, uint32_t data) {
    switch (event) {
        case 0x00: // 连接建立
            g_rs485_slave_adapter.master_connected = true;
            break;
        case 0x01: // 连接断开
            g_rs485_slave_adapter.master_connected = false;
            break;
        case 0x02: // 心跳事件
            // 由协议层处理
            break;
    }
}

// 协议回调：同步请求
void RS485_SlaveAdapter_OnSync(uint8_t* data, uint8_t length) {
    // Slave处理Master的同步请求
    if (length > 0 && data[0] == SYNC_FIELD_ALL) {
        // 发送所有数据
        RS485_SlaveAdapter_SendSync(SYNC_FIELD_ALL);
    } else {
        // 发送指定数据
        uint32_t fields = 0;
        for (int i = 0; i < length; i++) {
            fields |= (1 << data[i]);
        }
        RS485_SlaveAdapter_SendSync(fields);
    }
}

// 协议回调：门控命令
void RS485_SlaveAdapter_OnDoorCmd(uint8_t cmd, uint8_t* data, uint8_t length) {
    switch (cmd) {
        case RS485_CMD_DOOR_OPEN:
            LocalBlackboard_PushEvent(EVENT_OPEN_DOOR, 0);
            break;
        case RS485_CMD_DOOR_CLOSE:
            LocalBlackboard_PushEvent(EVENT_CLOSE_DOOR, 0);
            break;
        case RS485_CMD_DOOR_STOP:
            // 停止门操作
            break;
    }
    
    // 发送ACK
    RS485_Frame_t frame;
    RS485_Protocol_BuildAckFrame(&frame, cmd);
    frame.cmd = RS485_Protocol_GetCommand(&g_rs485_slave_adapter.protocol, RS485_CMD_ACK);
    RS485_Core_SendRawFrame(&g_rs485_slave_adapter.core, &frame);
}

// 协议回调：传感器数据（Slave通常不接收）
void RS485_SlaveAdapter_OnSensorData(uint8_t* data, uint8_t length) {
    // Slave通常不处理传感器数据
}

// 协议回调：键盘数据（Slave通常不接收）
void RS485_SlaveAdapter_OnKeypadData(uint8_t floor) {
    // Slave通常不处理键盘数据
}

// 协议回调：心跳
void RS485_SlaveAdapter_OnHeartbeat(uint32_t timestamp) {
    // 更新连接状态
    g_rs485_slave_adapter.master_last_seen = HAL_GetTick();
    
    // 响应心跳
    RS485_Frame_t frame;
    RS485_Protocol_BuildAckFrame(&frame, RS485_CMD_HEARTBEAT);
    frame.cmd = RS485_Protocol_GetCommand(&g_rs485_slave_adapter.protocol, RS485_CMD_ACK);
    RS485_Core_SendRawFrame(&g_rs485_slave_adapter.core, &frame);
}

// 协议回调：错误
void RS485_SlaveAdapter_OnProtocolError(uint16_t error_code) {
    g_rs485_slave_adapter.blackboard->error_code = error_code;
    LocalBlackboard_PushEvent(EVENT_ERROR, error_code);
}

// UART回调（需要在主程序中调用）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        RS485_Core_RxCallback(&g_rs485_slave_adapter.core);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        RS485_Core_TxCompleteCallback(&g_rs485_slave_adapter.core);
    }
}