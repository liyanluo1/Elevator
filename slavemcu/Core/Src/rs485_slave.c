#include "rs485_slave.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

// 全局RS485从机实例
RS485_Slave_t g_rs485_slave;

// 初始化RS485从机
void RS485_Slave_Init(void) {
    // 清零结构体
    memset(&g_rs485_slave, 0, sizeof(RS485_Slave_t));
    
    // 初始化CRC表
    RS485_Slave_InitCRCTable();
    
    // 设置初始状态
    g_rs485_slave.tx_state = TX_STATE_IDLE;
    g_rs485_slave.rx_state = RX_STATE_IDLE;
    g_rs485_slave.master_connected = false;
    
    // 初始化GPIO（RS485方向控制）
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = RS485_DE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_DE_GPIO_Port, &GPIO_InitStruct);
    
    // 默认接收模式
    RS485_Slave_SetMode(false);
    
    // 记录初始时间
    g_rs485_slave.last_rx_time = HAL_GetTick();
    g_rs485_slave.last_heartbeat_time = HAL_GetTick();
}

// 主处理函数
void RS485_Slave_Handler(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 检查连接超时
    if (current_time - g_rs485_slave.last_rx_time > RS485_TIMEOUT * 2) {
        if (g_rs485_slave.master_connected) {
            g_rs485_slave.master_connected = false;
            LocalBlackboard_PushEvent(EVENT_SYNC_TIMEOUT, 0);
        }
    }
    
    // 处理发送状态机
    switch (g_rs485_slave.tx_state) {
        case TX_STATE_WAIT_ACK:
            // 检查ACK超时
            if (current_time - g_rs485_slave.last_tx_time > RS485_TIMEOUT) {
                g_rs485_slave.retry_count++;
                if (g_rs485_slave.retry_count >= RS485_RETRY_MAX) {
                    g_rs485_slave.tx_state = TX_STATE_ERROR;
                    g_rs485_slave.comm_error_count++;
                } else {
                    // 重试发送
                    g_rs485_slave.tx_state = TX_STATE_IDLE;
                    RS485_Slave_TransmitFrame();
                }
            }
            break;
            
        case TX_STATE_ERROR:
            // 错误处理
            g_rs485_slave.tx_state = TX_STATE_IDLE;
            LocalBlackboard_PushEvent(EVENT_ERROR, 0x100); // RS485错误
            break;
            
        default:
            break;
    }
    
    // 检查是否需要发送心跳
    if (current_time - g_rs485_slave.last_heartbeat_time > 1000) { // 每秒一次
        RS485_Slave_SendHeartbeat();
    }
    
    // 检查黑板同步
    RS485_Slave_CheckSync();
}

// 发送同步数据
bool RS485_Slave_SendSync(uint32_t fields) {
    if (g_rs485_slave.tx_state != TX_STATE_IDLE) {
        return false;
    }
    
    uint8_t data[16];
    uint8_t index = 0;
    
    // 打包需要同步的数据
    if (fields & SYNC_FIELD_TARGET_FLOOR) {
        data[index++] = SYNC_TARGET_FLOOR;
        data[index++] = g_local_bb.target_floor;
    }
    
    if (fields & SYNC_FIELD_DOOR) {
        data[index++] = SYNC_DOOR_STATE;
        data[index++] = (uint8_t)g_local_bb.door;
    }
    
    if (fields & SYNC_FIELD_CURRENT_FLOOR) {
        data[index++] = SYNC_CURRENT_FLOOR;
        data[index++] = g_local_bb.current_floor;
    }
    
    if (fields & SYNC_FIELD_ERROR) {
        data[index++] = SYNC_ERROR_CODE;
        data[index++] = (uint8_t)(g_local_bb.error_code & 0xFF);
        data[index++] = (uint8_t)((g_local_bb.error_code >> 8) & 0xFF);
    }
    
    if (fields & SYNC_FIELD_POSITION_OFFSET) {
        data[index++] = SYNC_POSITION_OFFSET;
        data[index++] = (uint8_t)(g_local_bb.position_offset & 0xFF);
        data[index++] = (uint8_t)((g_local_bb.position_offset >> 8) & 0xFF);
    }
    
    return RS485_Slave_BuildFrame(CMD_SYNC_DATA, data, index);
}

// 发送传感器数据
bool RS485_Slave_SendSensorData(uint8_t floor, int offset) {
    if (g_rs485_slave.tx_state != TX_STATE_IDLE) {
        return false;
    }
    
    uint8_t data[4];
    data[0] = floor;
    data[1] = (uint8_t)(offset & 0xFF);
    data[2] = (uint8_t)((offset >> 8) & 0xFF);
    data[3] = g_local_bb.trigger_count;
    
    return RS485_Slave_BuildFrame(CMD_SENSOR_DATA, data, 4);
}

// 发送键盘数据
bool RS485_Slave_SendKeyboardData(uint8_t floor) {
    if (g_rs485_slave.tx_state != TX_STATE_IDLE) {
        return false;
    }
    
    uint8_t data[1];
    data[0] = floor;
    
    return RS485_Slave_BuildFrame(CMD_KEYBOARD_DATA, data, 1);
}

// 发送门状态
bool RS485_Slave_SendDoorStatus(DoorState_t state) {
    if (g_rs485_slave.tx_state != TX_STATE_IDLE) {
        return false;
    }
    
    uint8_t data[2];
    data[0] = (uint8_t)state;
    data[1] = g_local_bb.servo_position;
    
    return RS485_Slave_BuildFrame(CMD_DOOR_COMMAND, data, 2);
}

// 发送心跳
bool RS485_Slave_SendHeartbeat(void) {
    if (g_rs485_slave.tx_state != TX_STATE_IDLE) {
        return false;
    }
    
    uint8_t data[4];
    uint32_t uptime = HAL_GetTick();
    data[0] = (uint8_t)(uptime & 0xFF);
    data[1] = (uint8_t)((uptime >> 8) & 0xFF);
    data[2] = (uint8_t)((uptime >> 16) & 0xFF);
    data[3] = (uint8_t)((uptime >> 24) & 0xFF);
    
    g_rs485_slave.last_heartbeat_time = HAL_GetTick();
    
    return RS485_Slave_BuildFrame(CMD_HEARTBEAT, data, 4);
}

// 发送错误报告
bool RS485_Slave_SendError(int error_code) {
    if (g_rs485_slave.tx_state != TX_STATE_IDLE) {
        return false;
    }
    
    uint8_t data[2];
    data[0] = (uint8_t)(error_code & 0xFF);
    data[1] = (uint8_t)((error_code >> 8) & 0xFF);
    
    return RS485_Slave_BuildFrame(CMD_ERROR, data, 2);
}

// 发送ACK
bool RS485_Slave_SendACK(uint8_t cmd) {
    if (g_rs485_slave.tx_state != TX_STATE_IDLE) {
        return false;
    }
    
    uint8_t data[1];
    data[0] = cmd;
    
    return RS485_Slave_BuildFrame(CMD_ACK, data, 1);
}

// 发送NACK
bool RS485_Slave_SendNACK(uint8_t cmd, uint8_t reason) {
    if (g_rs485_slave.tx_state != TX_STATE_IDLE) {
        return false;
    }
    
    uint8_t data[2];
    data[0] = cmd;
    data[1] = reason;
    
    return RS485_Slave_BuildFrame(CMD_NACK, data, 2);
}

// 处理接收到的帧
void RS485_Slave_ProcessRxFrame(RS485_Frame_t* frame) {
    // 验证CRC
    uint16_t calc_crc = RS485_Slave_CalculateCRC(&frame->cmd, frame->length + 1);
    if (calc_crc != frame->crc) {
        // CRC错误，发送NACK
        RS485_Slave_SendNACK(frame->cmd, 0x01); // CRC错误
        return;
    }
    
    // 更新连接状态
    g_rs485_slave.master_connected = true;
    g_rs485_slave.last_rx_time = HAL_GetTick();
    
    // 处理命令
    RS485_Slave_ProcessCommand(frame->cmd, frame->data, frame->length);
}

// 处理命令
void RS485_Slave_ProcessCommand(uint8_t cmd, uint8_t* data, uint8_t length) {
    switch (cmd) {
        case CMD_SYNC_REQUEST:
            // Master请求同步
            if (length > 0 && data[0] == SYNC_ALL) {
                // 发送所有数据
                RS485_Slave_SendSync(0xFFFFFFFF);
            } else {
                // 发送指定数据
                uint32_t fields = 0;
                for (int i = 0; i < length; i++) {
                    switch (data[i]) {
                        case SYNC_TARGET_FLOOR:
                            fields |= SYNC_FIELD_TARGET_FLOOR;
                            break;
                        case SYNC_DOOR_STATE:
                            fields |= SYNC_FIELD_DOOR;
                            break;
                        case SYNC_CURRENT_FLOOR:
                            fields |= SYNC_FIELD_CURRENT_FLOOR;
                            break;
                        case SYNC_ERROR_CODE:
                            fields |= SYNC_FIELD_ERROR;
                            break;
                        case SYNC_POSITION_OFFSET:
                            fields |= SYNC_FIELD_POSITION_OFFSET;
                            break;
                    }
                }
                RS485_Slave_SendSync(fields);
            }
            break;
            
        case CMD_DOOR_COMMAND:
            // 门控制命令
            if (length > 0) {
                switch (data[0]) {
                    case 0x01: // 开门
                        LocalBlackboard_PushEvent(EVENT_OPEN_DOOR, 0);
                        break;
                    case 0x02: // 关门
                        LocalBlackboard_PushEvent(EVENT_CLOSE_DOOR, 0);
                        break;
                }
                RS485_Slave_SendACK(cmd);
            }
            break;
            
        case CMD_HEARTBEAT:
            // 响应心跳
            RS485_Slave_SendACK(cmd);
            break;
            
        case CMD_ACK:
            // 收到确认
            if (g_rs485_slave.tx_state == TX_STATE_WAIT_ACK) {
                g_rs485_slave.tx_state = TX_STATE_COMPLETE;
                g_rs485_slave.retry_count = 0;
            }
            break;
            
        case CMD_NACK:
            // 收到否认
            if (g_rs485_slave.tx_state == TX_STATE_WAIT_ACK) {
                g_rs485_slave.tx_state = TX_STATE_ERROR;
                g_rs485_slave.comm_error_count++;
            }
            break;
            
        default:
            // 未知命令
            RS485_Slave_SendNACK(cmd, 0x02); // 未知命令
            break;
    }
}

// 构建数据帧
bool RS485_Slave_BuildFrame(RS485_CommandType_t cmd, uint8_t* data, uint8_t length) {
    if (length > MAX_DATA_LENGTH) {
        return false;
    }
    
    // 构建帧
    g_rs485_slave.tx_frame.start = FRAME_START;
    g_rs485_slave.tx_frame.length = length;
    g_rs485_slave.tx_frame.cmd = cmd;
    
    if (length > 0 && data != NULL) {
        memcpy(g_rs485_slave.tx_frame.data, data, length);
    }
    
    // 计算CRC（包括cmd和data）
    uint8_t crc_data[MAX_DATA_LENGTH + 1];
    crc_data[0] = cmd;
    memcpy(&crc_data[1], data, length);
    g_rs485_slave.tx_frame.crc = RS485_Slave_CalculateCRC(crc_data, length + 1);
    
    g_rs485_slave.tx_frame.end = FRAME_END;
    
    // 准备发送缓冲区
    uint8_t index = 0;
    g_rs485_slave.tx_buffer[index++] = g_rs485_slave.tx_frame.start;
    g_rs485_slave.tx_buffer[index++] = g_rs485_slave.tx_frame.length;
    g_rs485_slave.tx_buffer[index++] = g_rs485_slave.tx_frame.cmd;
    
    if (length > 0) {
        memcpy(&g_rs485_slave.tx_buffer[index], g_rs485_slave.tx_frame.data, length);
        index += length;
    }
    
    g_rs485_slave.tx_buffer[index++] = (uint8_t)(g_rs485_slave.tx_frame.crc >> 8);
    g_rs485_slave.tx_buffer[index++] = (uint8_t)(g_rs485_slave.tx_frame.crc & 0xFF);
    g_rs485_slave.tx_buffer[index++] = g_rs485_slave.tx_frame.end;
    
    g_rs485_slave.tx_length = index;
    
    // 开始发送
    return RS485_Slave_TransmitFrame();
}

// 发送帧
bool RS485_Slave_TransmitFrame(void) {
    if (g_rs485_slave.tx_state != TX_STATE_IDLE) {
        return false;
    }
    
    // 切换到发送模式
    RS485_Slave_SetMode(true);
    
    // 发送数据
    HAL_UART_Transmit_IT(&huart2, g_rs485_slave.tx_buffer, g_rs485_slave.tx_length);
    
    g_rs485_slave.tx_state = TX_STATE_SENDING;
    g_rs485_slave.last_tx_time = HAL_GetTick();
    
    return true;
}

// 设置RS485收发模式
void RS485_Slave_SetMode(bool transmit) {
    if (transmit) {
        HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
    }
    HAL_Delay(1); // 短暂延时确保切换稳定
}

// 计算CRC
uint16_t RS485_Slave_CalculateCRC(uint8_t* data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    
    for (uint8_t i = 0; i < length; i++) {
        crc = (crc >> 8) ^ g_rs485_slave.crc_table[(crc ^ data[i]) & 0xFF];
    }
    
    return crc;
}

// 初始化CRC表
void RS485_Slave_InitCRCTable(void) {
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = 0;
        uint16_t c = i;
        
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc ^ c) & 0x0001) {
                crc = (crc >> 1) ^ CRC_POLY;
            } else {
                crc = crc >> 1;
            }
            c = c >> 1;
        }
        
        g_rs485_slave.crc_table[i] = crc;
    }
}

// UART接收回调
void RS485_Slave_UART_RxCallback(uint8_t data) {
    switch (g_rs485_slave.rx_state) {
        case RX_STATE_IDLE:
            if (data == FRAME_START) {
                g_rs485_slave.rx_state = RX_STATE_LENGTH;
                g_rs485_slave.rx_index = 0;
            }
            break;
            
        case RX_STATE_LENGTH:
            g_rs485_slave.rx_frame.length = data;
            if (data <= MAX_DATA_LENGTH) {
                g_rs485_slave.rx_state = RX_STATE_CMD;
                g_rs485_slave.rx_data_count = 0;
            } else {
                g_rs485_slave.rx_state = RX_STATE_IDLE; // 长度错误
            }
            break;
            
        case RX_STATE_CMD:
            g_rs485_slave.rx_frame.cmd = data;
            if (g_rs485_slave.rx_frame.length > 0) {
                g_rs485_slave.rx_state = RX_STATE_DATA;
            } else {
                g_rs485_slave.rx_state = RX_STATE_CRC_HIGH;
            }
            break;
            
        case RX_STATE_DATA:
            g_rs485_slave.rx_frame.data[g_rs485_slave.rx_data_count++] = data;
            if (g_rs485_slave.rx_data_count >= g_rs485_slave.rx_frame.length) {
                g_rs485_slave.rx_state = RX_STATE_CRC_HIGH;
            }
            break;
            
        case RX_STATE_CRC_HIGH:
            g_rs485_slave.rx_frame.crc = (uint16_t)(data << 8);
            g_rs485_slave.rx_state = RX_STATE_CRC_LOW;
            break;
            
        case RX_STATE_CRC_LOW:
            g_rs485_slave.rx_frame.crc |= data;
            g_rs485_slave.rx_state = RX_STATE_END;
            break;
            
        case RX_STATE_END:
            if (data == FRAME_END) {
                // 帧接收完成，处理数据
                RS485_Slave_ProcessRxFrame(&g_rs485_slave.rx_frame);
            }
            g_rs485_slave.rx_state = RX_STATE_IDLE;
            break;
            
        default:
            g_rs485_slave.rx_state = RX_STATE_IDLE;
            break;
    }
}

// UART发送完成回调
void RS485_Slave_UART_TxCompleteCallback(void) {
    // 切换回接收模式
    RS485_Slave_SetMode(false);
    
    if (g_rs485_slave.tx_state == TX_STATE_SENDING) {
        g_rs485_slave.tx_state = TX_STATE_WAIT_ACK;
    }
}

// UART错误回调
void RS485_Slave_UART_ErrorCallback(void) {
    g_rs485_slave.comm_error_count++;
    g_rs485_slave.rx_state = RX_STATE_IDLE;
    g_rs485_slave.tx_state = TX_STATE_IDLE;
    
    // 复位接收模式
    RS485_Slave_SetMode(false);
}

// 查询连接状态
bool RS485_Slave_IsConnected(void) {
    return g_rs485_slave.master_connected;
}

// 获取最后接收时间
uint32_t RS485_Slave_GetLastRxTime(void) {
    return g_rs485_slave.last_rx_time;
}

// 获取错误计数
uint16_t RS485_Slave_GetErrorCount(void) {
    return g_rs485_slave.comm_error_count;
}

// 检查同步需求
void RS485_Slave_CheckSync(void) {
    if (LocalBlackboard_NeedsSync()) {
        uint32_t fields = LocalBlackboard_GetSyncFields();
        if (RS485_Slave_SendSync(fields)) {
            LocalBlackboard_ClearSyncFlags();
        }
    }
}

// 从黑板更新数据
void RS485_Slave_UpdateFromBlackboard(void) {
    // 检查各种状态变化并发送相应消息
    static DoorState_t last_door_state = DOOR_CLOSED;
    static uint8_t last_floor = 1;
    
    // 检查门状态变化
    if (g_local_bb.door != last_door_state) {
        RS485_Slave_SendDoorStatus(g_local_bb.door);
        last_door_state = g_local_bb.door;
    }
    
    // 检查楼层变化
    if (g_local_bb.current_floor != last_floor) {
        RS485_Slave_SendSensorData(g_local_bb.current_floor, g_local_bb.position_offset);
        last_floor = g_local_bb.current_floor;
    }
}