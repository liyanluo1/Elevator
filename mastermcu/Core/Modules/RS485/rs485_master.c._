#include "rs485_master.h"
#include "usart.h"
#include <string.h>

// 外部UART句柄
extern UART_HandleTypeDef huart3;

// 全局Master实例
RS485_Master_t g_rs485_master;

// 初始化Master
void RS485_Master_Init(Blackboard_t* blackboard) {
    // 清零结构体
    memset(&g_rs485_master, 0, sizeof(RS485_Master_t));
    
    // 保存黑板引用
    g_rs485_master.blackboard = blackboard;
    
    // 配置硬件
    RS485_Config_t config = {
        .de_gpio_port = GPIOB,
        .de_pin = GPIO_PIN_0,
        .uart_handle = &huart3,
        .baudrate = 115200,
        .timeout_ms = 200,
        .retry_max = 3,
        .heartbeat_interval = 100
    };
    
    // 初始化核心
    RS485_Core_Init(&g_rs485_master.core, &config);
    
    // 设置核心回调
    g_rs485_master.core.on_frame_received = RS485_Master_OnFrameReceived;
    g_rs485_master.core.on_error = RS485_Master_OnError;
    g_rs485_master.core.on_event = RS485_Master_OnEvent;
    
    // 配置协议
    RS485_ProtocolConfig_t protocol_config = {
        .cmd_offset = MASTER_CMD_OFFSET,
        .use_ack = true,
        .use_heartbeat = true,
        .sync_interval = 200,
        .is_master = true
    };
    
    // 初始化协议
    RS485_Protocol_Init(&g_rs485_master.protocol, &protocol_config);
    
    // 设置协议回调
    g_rs485_master.protocol.on_sync = RS485_Master_OnSync;
    g_rs485_master.protocol.on_door_cmd = RS485_Master_OnDoorCmd;
    g_rs485_master.protocol.on_sensor_data = RS485_Master_OnSensorData;
    g_rs485_master.protocol.on_keypad_data = RS485_Master_OnKeypadData;
    g_rs485_master.protocol.on_heartbeat = RS485_Master_OnHeartbeat;
    g_rs485_master.protocol.on_error = RS485_Master_OnProtocolError;
}

// 主处理函数
void RS485_Master_Handler(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 处理核心
    RS485_Core_Handler(&g_rs485_master.core);
    
    // 检查同步需求
    if (current_time - g_rs485_master.last_sync_time > 200 || g_rs485_master.sync_pending) {
        RS485_Master_SendSync();
        g_rs485_master.last_sync_time = current_time;
        g_rs485_master.sync_pending = false;
    }
    
    // 发送心跳
    if (current_time - g_rs485_master.core.last_heartbeat_tx > g_rs485_master.core.config.heartbeat_interval) {
        RS485_Master_SendHeartbeat();
    }
    
    // 更新连接状态
    if (current_time - g_rs485_master.slave_last_seen > 5000) {
        if (g_rs485_master.slave_connected) {
            g_rs485_master.slave_connected = false;
            g_rs485_master.blackboard->comm.slave_connected = false;
            g_rs485_master.blackboard->rs485_sync_delay_flag = true;
            Blackboard_PushEvent(EVENT_COMM_TIMEOUT, 0);
        }
    } else {
        g_rs485_master.slave_connected = true;
        g_rs485_master.blackboard->comm.slave_connected = true;
        g_rs485_master.blackboard->rs485_sync_delay_flag = false;
    }
}

// 发送同步数据
bool RS485_Master_SendSync(void) {
    uint8_t sync_data[16];
    uint8_t index = 0;
    
    // 打包同步数据
    sync_data[index++] = g_rs485_master.blackboard->target_floor;
    sync_data[index++] = g_rs485_master.blackboard->current_floor;
    sync_data[index++] = g_rs485_master.blackboard->dir;
    sync_data[index++] = g_rs485_master.blackboard->door.is_open;
    sync_data[index++] = g_rs485_master.blackboard->door.is_closed;
    sync_data[index++] = g_rs485_master.blackboard->state;
    RS485_Protocol_PackUint16(&sync_data[index], g_rs485_master.blackboard->error_code);
    index += 2;
    RS485_Protocol_PackUint16(&sync_data[index], g_rs485_master.blackboard->position_offset);
    index += 2;
    
    // 构建帧
    RS485_Frame_t frame;
    uint8_t length = index;
    RS485_Protocol_BuildSyncFrame(&frame, SYNC_FIELD_ALL, sync_data, &length);
    
    // 获取实际命令
    frame.cmd = RS485_Protocol_GetCommand(&g_rs485_master.protocol, RS485_CMD_SYNC);
    
    // 发送
    return RS485_Core_SendRawFrame(&g_rs485_master.core, &frame) == RS485_OK;
}

// 发送门控命令
bool RS485_Master_SendDoorCommand(uint8_t door_cmd) {
    RS485_Frame_t frame;
    RS485_Protocol_BuildDoorFrame(&frame, 
        RS485_Protocol_GetCommand(&g_rs485_master.protocol, RS485_CMD_DOOR_OPEN + door_cmd - 1), 
        &door_cmd);
    
    return RS485_Core_SendRawFrame(&g_rs485_master.core, &frame) == RS485_OK;
}

// 请求传感器状态
bool RS485_Master_SendSensorRequest(void) {
    uint8_t cmd = RS485_Protocol_GetCommand(&g_rs485_master.protocol, RS485_CMD_SENSOR_STATUS);
    return RS485_Core_SendFrame(&g_rs485_master.core, cmd, NULL, 0, false) == RS485_OK;
}

// 发送心跳
bool RS485_Master_SendHeartbeat(void) {
    RS485_Frame_t frame;
    uint32_t timestamp = HAL_GetTick();
    RS485_Protocol_BuildHeartbeatFrame(&frame, timestamp);
    
    frame.cmd = RS485_Protocol_GetCommand(&g_rs485_master.protocol, RS485_CMD_HEARTBEAT);
    
    g_rs485_master.core.last_heartbeat_tx = timestamp;
    
    return RS485_Core_SendRawFrame(&g_rs485_master.core, &frame) == RS485_OK;
}

// 查询连接状态
bool RS485_Master_IsSlaveConnected(void) {
    return g_rs485_master.slave_connected;
}

// 获取最后接收时间
uint32_t RS485_Master_GetLastRxTime(void) {
    return g_rs485_master.slave_last_seen;
}

// 帧接收回调
void RS485_Master_OnFrameReceived(RS485_Frame_t* frame) {
    // 更新接收时间
    g_rs485_master.slave_last_seen = HAL_GetTick();
    g_rs485_master.blackboard->comm.last_rx_time = g_rs485_master.slave_last_seen;
    
    // 协议处理
    RS485_Protocol_ProcessFrame(&g_rs485_master.protocol, frame);
}

// 错误回调
void RS485_Master_OnError(RS485_Error_t error) {
    g_rs485_master.blackboard->comm.error_count++;
    
    switch (error) {
        case RS485_ERROR_TIMEOUT:
            Blackboard_PushEvent(EVENT_SYNC_TIMEOUT, 0);
            break;
        case RS485_ERROR_CRC:
            // CRC错误已在核心层统计
            break;
        default:
            Blackboard_PushEvent(EVENT_ERROR, error);
            break;
    }
}

// 事件回调
void RS485_Master_OnEvent(uint8_t event, uint32_t data) {
    switch (event) {
        case 0x00: // 连接建立
            g_rs485_master.slave_connected = true;
            break;
        case 0x01: // 连接断开
            g_rs485_master.slave_connected = false;
            break;
        case 0x02: // 心跳事件
            // 由协议层处理
            break;
    }
}

// 协议回调：同步数据
void RS485_Master_OnSync(uint8_t* data, uint8_t length) {
    // Master通常不接收同步数据，但可以处理Slave的状态反馈
    if (length >= 10) {
        // 可以解析Slave的状态数据
    }
}

// 协议回调：门控命令
void RS485_Master_OnDoorCmd(uint8_t cmd, uint8_t* data, uint8_t length) {
    if (cmd == RS485_CMD_DOOR_STATUS && length >= 3) {
        g_rs485_master.blackboard->door.is_open = data[0];
        g_rs485_master.blackboard->door.is_closed = data[1];
        g_rs485_master.blackboard->door.position = data[2];
        
        if (g_rs485_master.blackboard->door.is_open) {
            Blackboard_PushEvent(EVENT_DOOR_OPENED, g_rs485_master.blackboard->current_floor);
        } else if (g_rs485_master.blackboard->door.is_closed) {
            Blackboard_PushEvent(EVENT_DOOR_CLOSED, g_rs485_master.blackboard->current_floor);
        }
    }
}

// 协议回调：传感器数据
void RS485_Master_OnSensorData(uint8_t* data, uint8_t length) {
    if (length >= MAX_FLOORS) {
        for (int i = 0; i < MAX_FLOORS; i++) {
            if (data[i] && !g_rs485_master.blackboard->sensors.sensor_triggered[i]) {
                g_rs485_master.blackboard->sensors.sensor_triggered[i] = true;
                Blackboard_PushEvent(EVENT_SENSOR_TRIGGERED, i);
            } else if (!data[i]) {
                g_rs485_master.blackboard->sensors.sensor_triggered[i] = false;
            }
        }
    }
}

// 协议回调：键盘数据
void RS485_Master_OnKeypadData(uint8_t floor) {
    if (floor >= 1 && floor <= 3) {
        g_rs485_master.blackboard->target_floor = floor - 1;
        Blackboard_SetPendingCall(floor, true);
    }
}

// 协议回调：心跳
void RS485_Master_OnHeartbeat(uint32_t timestamp) {
    // 更新连接状态
    g_rs485_master.slave_last_seen = HAL_GetTick();
}

// 协议回调：错误
void RS485_Master_OnProtocolError(uint16_t error_code) {
    g_rs485_master.blackboard->error_code = error_code;
    Blackboard_PushEvent(EVENT_ERROR, 0);
}

// UART回调（需要在主程序中调用）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        RS485_Core_RxCallback(&g_rs485_master.core);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        RS485_Core_TxCompleteCallback(&g_rs485_master.core);
    }
}