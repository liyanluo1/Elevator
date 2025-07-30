#include "rs485_protocol.h"
#include "blackboard.h"
#include "usart.h"
#include <string.h>

// 外部UART句柄
extern UART_HandleTypeDef huart3;

// 全局RS485控制实例
RS485_Control_t g_rs485;

// UART接收字节
static uint8_t uart_rx_byte;

// CRC16-CCITT多项式
#define CRC16_POLY 0x1021

// 初始化RS485
void RS485_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIO时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // 配置DE引脚
    GPIO_InitStruct.Pin = RS485_DE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RS485_DE_GPIO_Port, &GPIO_InitStruct);
    
    // 默认接收模式
    RS485_SetMode(false);
    
    // 初始化控制结构体
    memset(&g_rs485, 0, sizeof(RS485_Control_t));
    g_rs485.tx_buffer.state = TX_STATE_IDLE;
    g_rs485.rx_buffer.state = RX_STATE_IDLE;
    
    // 初始化CRC表
    RS485_InitCRC();
    
    // 开启UART接收中断
    HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
}

// 初始化CRC查找表
void RS485_InitCRC(void) {
    for (int i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
        g_rs485.crc_table[i] = crc;
    }
}

// 计算CRC16
uint16_t RS485_CalculateCRC(uint8_t* data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    
    for (int i = 0; i < length; i++) {
        uint8_t index = (crc >> 8) ^ data[i];
        crc = (crc << 8) ^ g_rs485.crc_table[index];
    }
    
    return crc;
}

// RS485主处理函数
void RS485_Handler(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 处理发送状态机
    RS485_ProcessTx();
    
    // 处理接收状态机
    RS485_ProcessRx();
    
    // 检查同步需求（每200ms或有变化时）
    if (current_time - g_rs485.last_sync_time > 200 || g_rs485.sync_pending) {
        RS485_SendSync();
        g_rs485.last_sync_time = current_time;
        g_rs485.sync_pending = false;
    }
    
    // 发送心跳
    if (current_time - g_rs485.last_heartbeat_time > HEARTBEAT_INTERVAL) {
        RS485_SendHeartbeat();
        g_rs485.last_heartbeat_time = current_time;
    }
    
    // 更新连接状态
    if (current_time - g_blackboard.comm.last_rx_time > 5000) {
        g_blackboard.comm.slave_connected = false;
        g_blackboard.rs485_sync_delay_flag = true;
        if (current_time - g_blackboard.comm.last_rx_time > 10000) {
            // 通信超时事件
            Blackboard_PushEvent(EVENT_COMM_TIMEOUT, 0);
        }
    } else {
        g_blackboard.comm.slave_connected = true;
        g_blackboard.rs485_sync_delay_flag = false;
    }
}

// 处理发送状态机
void RS485_ProcessTx(void) {
    TxBuffer_t* tx = &g_rs485.tx_buffer;
    uint32_t current_time = HAL_GetTick();
    
    switch (tx->state) {
        case TX_STATE_WAIT_ACK:
            // 检查超时
            if (current_time - tx->last_tx_time > RS485_TIMEOUT) {
                if (tx->retry_count < RS485_RETRY_MAX) {
                    tx->retry_count++;
                    tx->state = TX_STATE_SENDING;
                } else {
                    tx->state = TX_STATE_ERROR;
                    Blackboard_PushEvent(EVENT_SYNC_TIMEOUT, 0);
                }
            }
            break;
            
        case TX_STATE_ERROR:
            // 错误处理
            g_blackboard.comm.error_count++;
            tx->state = TX_STATE_IDLE;
            break;
            
        default:
            break;
    }
}

// 处理接收状态机
void RS485_ProcessRx(void) {
    RxBuffer_t* rx = &g_rs485.rx_buffer;
    uint32_t current_time = HAL_GetTick();
    
    // 接收超时重置
    if (rx->state != RX_STATE_IDLE && 
        current_time - rx->last_rx_time > 100) {
        rx->state = RX_STATE_IDLE;
    }
}

// 发送命令
bool RS485_SendCommand(CommandType_t cmd, uint8_t* data, uint8_t length) {
    TxBuffer_t* tx = &g_rs485.tx_buffer;
    
    // 检查是否空闲
    if (tx->state != TX_STATE_IDLE) {
        return false;
    }
    
    if (length > MAX_DATA_LENGTH) {
        return false;
    }
    
    // 构建帧
    tx->frame.start = FRAME_START;
    tx->frame.length = length;
    tx->frame.cmd = cmd;
    
    if (data && length > 0) {
        memcpy(tx->frame.data, data, length);
    }
    
    // 计算CRC
    uint8_t crc_data[2 + MAX_DATA_LENGTH];
    crc_data[0] = tx->frame.length;
    crc_data[1] = tx->frame.cmd;
    memcpy(&crc_data[2], tx->frame.data, length);
    tx->frame.crc = RS485_CalculateCRC(crc_data, 2 + length);
    
    tx->frame.end = FRAME_END;
    
    // 发送帧
    RS485_SetMode(true);  // 发送模式
    
    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(&huart3, &tx->frame.start, 1, RS485_TIMEOUT);
    if (status == HAL_OK) {
        status = HAL_UART_Transmit(&huart3, &tx->frame.length, 1, RS485_TIMEOUT);
    }
    if (status == HAL_OK) {
        status = HAL_UART_Transmit(&huart3, &tx->frame.cmd, 1, RS485_TIMEOUT);
    }
    if (status == HAL_OK && length > 0) {
        status = HAL_UART_Transmit(&huart3, tx->frame.data, length, RS485_TIMEOUT);
    }
    if (status == HAL_OK) {
        uint8_t crc_bytes[2];
        crc_bytes[0] = tx->frame.crc & 0xFF;
        crc_bytes[1] = (tx->frame.crc >> 8) & 0xFF;
        status = HAL_UART_Transmit(&huart3, crc_bytes, 2, RS485_TIMEOUT);
    }
    if (status == HAL_OK) {
        status = HAL_UART_Transmit(&huart3, &tx->frame.end, 1, RS485_TIMEOUT);
    }
    
    RS485_SetMode(false);  // 接收模式
    
    if (status == HAL_OK) {
        tx->state = TX_STATE_WAIT_ACK;
        tx->last_tx_time = HAL_GetTick();
        tx->retry_count = 0;
        tx->ack_received = false;
        g_blackboard.comm.last_tx_time = tx->last_tx_time;
        return true;
    } else {
        tx->state = TX_STATE_ERROR;
        return false;
    }
}

// 发送同步数据
bool RS485_SendSync(void) {
    uint8_t sync_data[16];
    uint8_t index = 0;
    
    // 打包需要同步的数据
    sync_data[index++] = g_blackboard.target_floor;
    sync_data[index++] = g_blackboard.current_floor;
    sync_data[index++] = g_blackboard.dir;
    sync_data[index++] = g_blackboard.door.is_open;
    sync_data[index++] = g_blackboard.door.is_closed;
    sync_data[index++] = g_blackboard.state;
    sync_data[index++] = g_blackboard.error_code & 0xFF;
    sync_data[index++] = (g_blackboard.error_code >> 8) & 0xFF;
    sync_data[index++] = g_blackboard.position_offset & 0xFF;
    sync_data[index++] = (g_blackboard.position_offset >> 8) & 0xFF;
    
    return RS485_SendCommand(CMD_SYNC, sync_data, index);
}

// 发送门控命令
bool RS485_SendDoorCommand(DoorCommand_t door_cmd) {
    uint8_t data[1] = {door_cmd};
    return RS485_SendCommand(CMD_DOOR_OPEN + door_cmd - 1, data, 1);
}

// 设置RS485模式
void RS485_SetMode(bool transmit) {
    if (transmit) {
        HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
    }
    HAL_Delay(1);  // 模式切换延时
}

// UART接收中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        RS485_UART_RxCallback(uart_rx_byte);
        HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
    }
}

// RS485 UART接收处理
void RS485_UART_RxCallback(uint8_t data) {
    RxBuffer_t* rx = &g_rs485.rx_buffer;
    rx->last_rx_time = HAL_GetTick();
    
    switch (rx->state) {
        case RX_STATE_IDLE:
            if (data == FRAME_START) {
                rx->frame.start = data;
                rx->state = RX_STATE_LENGTH;
                rx->data_index = 0;
            }
            break;
            
        case RX_STATE_LENGTH:
            rx->frame.length = data;
            if (data <= MAX_DATA_LENGTH) {
                rx->state = RX_STATE_CMD;
            } else {
                rx->state = RX_STATE_IDLE;
            }
            break;
            
        case RX_STATE_CMD:
            rx->frame.cmd = data;
            if (rx->frame.length > 0) {
                rx->state = RX_STATE_DATA;
                rx->data_index = 0;
            } else {
                rx->state = RX_STATE_CRC_LOW;
            }
            break;
            
        case RX_STATE_DATA:
            rx->frame.data[rx->data_index++] = data;
            if (rx->data_index >= rx->frame.length) {
                rx->state = RX_STATE_CRC_LOW;
            }
            break;
            
        case RX_STATE_CRC_LOW:
            rx->crc_received = data;
            rx->state = RX_STATE_CRC_HIGH;
            break;
            
        case RX_STATE_CRC_HIGH:
            rx->crc_received |= (data << 8);
            rx->state = RX_STATE_END;
            break;
            
        case RX_STATE_END:
            if (data == FRAME_END) {
                // 验证CRC
                uint8_t crc_data[2 + MAX_DATA_LENGTH];
                crc_data[0] = rx->frame.length;
                crc_data[1] = rx->frame.cmd;
                memcpy(&crc_data[2], rx->frame.data, rx->frame.length);
                uint16_t calc_crc = RS485_CalculateCRC(crc_data, 2 + rx->frame.length);
                
                if (calc_crc == rx->crc_received) {
                    RS485_ProcessRxFrame(&rx->frame);
                    g_blackboard.comm.last_rx_time = HAL_GetTick();
                    
                    // 处理ACK
                    if (rx->frame.cmd == CMD_ACK) {
                        g_rs485.tx_buffer.ack_received = true;
                        g_rs485.tx_buffer.state = TX_STATE_COMPLETE;
                    }
                }
            }
            rx->state = RX_STATE_IDLE;
            break;
    }
}

// 处理接收到的数据帧
void RS485_ProcessRxFrame(RS485_Frame_t* frame) {
    switch (frame->cmd) {
        case CMD_SYNC:
            // 同步Local BB数据到Global BB
            if (frame->length >= 10) {
                // 解析同步数据（注意：这是从Slave接收的）
                // 这里需要根据实际需求调整
            }
            break;
            
        case CMD_DOOR_STATUS:
            if (frame->length >= 3) {
                g_blackboard.door.is_open = frame->data[0];
                g_blackboard.door.is_closed = frame->data[1];
                g_blackboard.door.position = frame->data[2];
                
                if (g_blackboard.door.is_open) {
                    Blackboard_PushEvent(EVENT_DOOR_OPENED, g_blackboard.current_floor);
                } else if (g_blackboard.door.is_closed) {
                    Blackboard_PushEvent(EVENT_DOOR_CLOSED, g_blackboard.current_floor);
                }
            }
            break;
            
        case CMD_SENSOR_STATUS:
            if (frame->length >= MAX_FLOORS) {
                for (int i = 0; i < MAX_FLOORS; i++) {
                    if (frame->data[i] && !g_blackboard.sensors.sensor_triggered[i]) {
                        g_blackboard.sensors.sensor_triggered[i] = true;
                        Blackboard_PushEvent(EVENT_SENSOR_TRIGGERED, i);
                    } else if (!frame->data[i]) {
                        g_blackboard.sensors.sensor_triggered[i] = false;
                    }
                }
            }
            break;
            
        case CMD_KEYPAD_DATA:
            if (frame->length >= 1) {
                uint8_t floor = frame->data[0];
                if (floor >= 1 && floor <= 3) {
                    g_blackboard.target_floor = floor - 1;
                    Blackboard_SetPendingCall(floor, true);
                }
            }
            break;
            
        case CMD_ERROR:
            if (frame->length >= 2) {
                int error_code = frame->data[0] | (frame->data[1] << 8);
                g_blackboard.error_code = error_code;
                Blackboard_PushEvent(EVENT_ERROR, 0);
            }
            break;
    }
}

// 检查是否连接
bool RS485_IsConnected(void) {
    return g_blackboard.comm.slave_connected;
}

// 获取最后接收时间
uint32_t RS485_GetLastRxTime(void) {
    return g_blackboard.comm.last_rx_time;
}

// 检查同步是否延迟
bool RS485_IsSyncDelayed(void) {
    return g_blackboard.rs485_sync_delay_flag;
}

// 请求传感器状态
bool RS485_RequestSensorStatus(void) {
    return RS485_SendCommand(CMD_SENSOR_STATUS, NULL, 0);
}

// 发送心跳
bool RS485_SendHeartbeat(void) {
    uint8_t data[4];
    uint32_t uptime = HAL_GetTick();
    
    data[0] = (uptime >> 24) & 0xFF;
    data[1] = (uptime >> 16) & 0xFF;
    data[2] = (uptime >> 8) & 0xFF;
    data[3] = uptime & 0xFF;
    
    return RS485_SendCommand(CMD_HEARTBEAT, data, 4);
}