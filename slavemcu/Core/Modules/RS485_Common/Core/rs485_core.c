#include "rs485_core.h"
#include <string.h>

// 初始化RS485核心
RS485_Error_t RS485_Core_Init(RS485_Core_t* core, RS485_Config_t* config) {
    if (core == NULL || config == NULL) {
        return RS485_ERROR_UNKNOWN;
    }
    
    // 清零结构体
    memset(core, 0, sizeof(RS485_Core_t));
    
    // 复制配置
    memcpy(&core->config, config, sizeof(RS485_Config_t));
    
    // 设置默认值
    if (core->config.baudrate == 0) {
        core->config.baudrate = RS485_DEFAULT_BAUDRATE;
    }
    if (core->config.timeout_ms == 0) {
        core->config.timeout_ms = RS485_DEFAULT_TIMEOUT;
    }
    if (core->config.retry_max == 0) {
        core->config.retry_max = RS485_DEFAULT_RETRY_MAX;
    }
    if (core->config.heartbeat_interval == 0) {
        core->config.heartbeat_interval = RS485_HEARTBEAT_INTERVAL;
    }
    
    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = core->config.de_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(core->config.de_gpio_port, &GPIO_InitStruct);
    
    // 默认接收模式
    RS485_Core_SetMode(core, false);
    
    // 初始化状态
    core->tx_state = RS485_TX_IDLE;
    core->rx_state = RS485_RX_IDLE;
    core->is_connected = false;
    
    // 初始化CRC表
    RS485_Core_InitCRC(core);
    
    // 开启UART接收中断
    HAL_UART_Receive_IT(core->config.uart_handle, &core->uart_rx_byte, 1);
    
    return RS485_OK;
}

// 反初始化
void RS485_Core_DeInit(RS485_Core_t* core) {
    if (core == NULL) return;
    
    // 停止UART
    HAL_UART_AbortReceive_IT(core->config.uart_handle);
    HAL_UART_AbortTransmit_IT(core->config.uart_handle);
    
    // 复位GPIO
    HAL_GPIO_DeInit(core->config.de_gpio_port, core->config.de_pin);
    
    // 清零结构体
    memset(core, 0, sizeof(RS485_Core_t));
}

// 主处理函数
void RS485_Core_Handler(RS485_Core_t* core) {
    if (core == NULL) return;
    
    uint32_t current_time = HAL_GetTick();
    
    // 处理发送状态机
    RS485_Core_ProcessTxStateMachine(core);
    
    // 处理接收状态机
    RS485_Core_ProcessRxStateMachine(core);
    
    // 检查心跳
    RS485_Core_CheckHeartbeat(core);
    
    // 检查连接状态
    if (current_time - core->rx_timestamp > core->config.timeout_ms * 2) {
        if (core->is_connected) {
            core->is_connected = false;
            if (core->on_event) {
                core->on_event(0x01, 0); // 连接断开事件
            }
        }
    }
}

// 发送数据帧
RS485_Error_t RS485_Core_SendFrame(RS485_Core_t* core, uint8_t cmd, 
                                   uint8_t* data, uint8_t length, bool need_ack) {
    if (core == NULL) {
        return RS485_ERROR_UNKNOWN;
    }
    
    // 检查状态
    if (core->tx_state != RS485_TX_IDLE) {
        return RS485_ERROR_BUSY;
    }
    
    // 检查长度
    if (length > RS485_MAX_DATA_LENGTH) {
        return RS485_ERROR_FRAME;
    }
    
    // 构建帧
    core->tx_frame.start = RS485_FRAME_START;
    core->tx_frame.length = length;
    core->tx_frame.cmd = cmd;
    
    if (data && length > 0) {
        memcpy(core->tx_frame.data, data, length);
    }
    
    // 计算CRC
    uint8_t crc_data[RS485_MAX_DATA_LENGTH + 2];
    crc_data[0] = core->tx_frame.length;
    crc_data[1] = core->tx_frame.cmd;
    if (length > 0) {
        memcpy(&crc_data[2], core->tx_frame.data, length);
    }
    core->tx_frame.crc = RS485_Core_CalculateCRC(core, crc_data, 2 + length);
    
    core->tx_frame.end = RS485_FRAME_END;
    
    // 准备发送缓冲区
    uint16_t index = 0;
    core->tx_buffer[index++] = core->tx_frame.start;
    core->tx_buffer[index++] = core->tx_frame.length;
    core->tx_buffer[index++] = core->tx_frame.cmd;
    
    if (length > 0) {
        memcpy(&core->tx_buffer[index], core->tx_frame.data, length);
        index += length;
    }
    
    core->tx_buffer[index++] = (uint8_t)(core->tx_frame.crc & 0xFF);
    core->tx_buffer[index++] = (uint8_t)((core->tx_frame.crc >> 8) & 0xFF);
    core->tx_buffer[index++] = core->tx_frame.end;
    
    core->tx_length = index;
    core->wait_ack = need_ack;
    
    // 开始发送
    RS485_Core_TransmitFrame(core);
    
    return RS485_OK;
}

// 发送原始帧
RS485_Error_t RS485_Core_SendRawFrame(RS485_Core_t* core, RS485_Frame_t* frame) {
    if (core == NULL || frame == NULL) {
        return RS485_ERROR_UNKNOWN;
    }
    
    return RS485_Core_SendFrame(core, frame->cmd, frame->data, frame->length, true);
}

// 处理接收字节
void RS485_Core_ProcessRxByte(RS485_Core_t* core, uint8_t byte) {
    if (core == NULL) return;
    
    core->rx_timestamp = HAL_GetTick();
    
    switch (core->rx_state) {
        case RS485_RX_IDLE:
            if (byte == RS485_FRAME_START) {
                core->rx_frame.start = byte;
                core->rx_state = RS485_RX_LENGTH;
                core->rx_data_index = 0;
            }
            break;
            
        case RS485_RX_LENGTH:
            core->rx_frame.length = byte;
            if (byte <= RS485_MAX_DATA_LENGTH) {
                core->rx_state = RS485_RX_CMD;
            } else {
                core->rx_state = RS485_RX_IDLE;
                core->stats.error_count++;
            }
            break;
            
        case RS485_RX_CMD:
            core->rx_frame.cmd = byte;
            if (core->rx_frame.length > 0) {
                core->rx_state = RS485_RX_DATA;
                core->rx_data_index = 0;
            } else {
                core->rx_state = RS485_RX_CRC_LOW;
            }
            break;
            
        case RS485_RX_DATA:
            core->rx_frame.data[core->rx_data_index++] = byte;
            if (core->rx_data_index >= core->rx_frame.length) {
                core->rx_state = RS485_RX_CRC_LOW;
            }
            break;
            
        case RS485_RX_CRC_LOW:
            core->rx_crc_temp = byte;
            core->rx_state = RS485_RX_CRC_HIGH;
            break;
            
        case RS485_RX_CRC_HIGH:
            core->rx_crc_temp |= (byte << 8);
            core->rx_frame.crc = core->rx_crc_temp;
            core->rx_state = RS485_RX_END;
            break;
            
        case RS485_RX_END:
            if (byte == RS485_FRAME_END) {
                core->rx_frame.end = byte;
                RS485_Core_ProcessRxFrame(core);
                core->stats.rx_count++;
            } else {
                core->stats.error_count++;
            }
            core->rx_state = RS485_RX_IDLE;
            break;
            
        default:
            core->rx_state = RS485_RX_IDLE;
            break;
    }
}

// 处理接收到的帧
void RS485_Core_ProcessRxFrame(RS485_Core_t* core) {
    if (core == NULL) return;
    
    // 验证CRC
    uint8_t crc_data[RS485_MAX_DATA_LENGTH + 2];
    crc_data[0] = core->rx_frame.length;
    crc_data[1] = core->rx_frame.cmd;
    if (core->rx_frame.length > 0) {
        memcpy(&crc_data[2], core->rx_frame.data, core->rx_frame.length);
    }
    
    uint16_t calc_crc = RS485_Core_CalculateCRC(core, crc_data, 2 + core->rx_frame.length);
    
    if (calc_crc != core->rx_frame.crc) {
        core->stats.crc_error_count++;
        if (core->on_error) {
            core->on_error(RS485_ERROR_CRC);
        }
        return;
    }
    
    // 更新连接状态
    if (!core->is_connected) {
        core->is_connected = true;
        if (core->on_event) {
            core->on_event(0x00, 0); // 连接建立事件
        }
    }
    
    // 处理ACK
    if (core->wait_ack && (core->rx_frame.cmd == 0x07 || core->rx_frame.cmd == 0x60)) {
        core->tx_state = RS485_TX_COMPLETE;
        core->wait_ack = false;
    }
    
    // 回调用户处理函数
    if (core->on_frame_received) {
        core->on_frame_received(&core->rx_frame);
    }
}

// 设置收发模式
void RS485_Core_SetMode(RS485_Core_t* core, bool transmit) {
    if (core == NULL) return;
    
    if (transmit) {
        HAL_GPIO_WritePin(core->config.de_gpio_port, core->config.de_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(core->config.de_gpio_port, core->config.de_pin, GPIO_PIN_RESET);
    }
    HAL_Delay(1); // 模式切换延时
}

// 复位核心
void RS485_Core_Reset(RS485_Core_t* core) {
    if (core == NULL) return;
    
    core->tx_state = RS485_TX_IDLE;
    core->rx_state = RS485_RX_IDLE;
    core->retry_count = 0;
    core->wait_ack = false;
    
    RS485_Core_SetMode(core, false);
}

// 中止发送
void RS485_Core_AbortTx(RS485_Core_t* core) {
    if (core == NULL) return;
    
    HAL_UART_AbortTransmit_IT(core->config.uart_handle);
    core->tx_state = RS485_TX_IDLE;
    RS485_Core_SetMode(core, false);
}

// 查询连接状态
bool RS485_Core_IsConnected(RS485_Core_t* core) {
    return (core != NULL) ? core->is_connected : false;
}

// 查询忙状态
bool RS485_Core_IsBusy(RS485_Core_t* core) {
    return (core != NULL) ? (core->tx_state != RS485_TX_IDLE) : false;
}

// 获取发送状态
RS485_TxState_t RS485_Core_GetTxState(RS485_Core_t* core) {
    return (core != NULL) ? core->tx_state : RS485_TX_IDLE;
}

// 获取接收状态
RS485_RxState_t RS485_Core_GetRxState(RS485_Core_t* core) {
    return (core != NULL) ? core->rx_state : RS485_RX_IDLE;
}

// 获取统计信息
RS485_Stats_t* RS485_Core_GetStats(RS485_Core_t* core) {
    return (core != NULL) ? &core->stats : NULL;
}

// 重置统计信息
void RS485_Core_ResetStats(RS485_Core_t* core) {
    if (core == NULL) return;
    
    memset(&core->stats, 0, sizeof(RS485_Stats_t));
}

// 初始化CRC表
void RS485_Core_InitCRC(RS485_Core_t* core) {
    if (core == NULL || core->crc_table_initialized) return;
    
    for (int i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ RS485_CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
        core->crc_table[i] = crc;
    }
    
    core->crc_table_initialized = true;
}

// 计算CRC
uint16_t RS485_Core_CalculateCRC(RS485_Core_t* core, uint8_t* data, uint8_t length) {
    if (core == NULL || data == NULL) return 0;
    
    uint16_t crc = 0xFFFF;
    
    for (int i = 0; i < length; i++) {
        uint8_t index = (crc >> 8) ^ data[i];
        crc = (crc << 8) ^ core->crc_table[index];
    }
    
    return crc;
}

// 处理发送状态机
void RS485_Core_ProcessTxStateMachine(RS485_Core_t* core) {
    if (core == NULL) return;
    
    uint32_t current_time = HAL_GetTick();
    
    switch (core->tx_state) {
        case RS485_TX_WAIT_ACK:
            if (core->wait_ack && (current_time - core->tx_timestamp > core->config.timeout_ms)) {
                if (core->retry_count < core->config.retry_max) {
                    core->retry_count++;
                    core->stats.retry_count++;
                    RS485_Core_TransmitFrame(core);
                } else {
                    core->tx_state = RS485_TX_ERROR;
                    core->stats.timeout_count++;
                    if (core->on_error) {
                        core->on_error(RS485_ERROR_TIMEOUT);
                    }
                }
            }
            break;
            
        case RS485_TX_COMPLETE:
            core->tx_state = RS485_TX_IDLE;
            core->retry_count = 0;
            break;
            
        case RS485_TX_ERROR:
            core->tx_state = RS485_TX_IDLE;
            core->retry_count = 0;
            core->stats.error_count++;
            break;
            
        default:
            break;
    }
}

// 处理接收状态机
void RS485_Core_ProcessRxStateMachine(RS485_Core_t* core) {
    if (core == NULL) return;
    
    uint32_t current_time = HAL_GetTick();
    
    // 接收超时重置
    if (core->rx_state != RS485_RX_IDLE && 
        (current_time - core->rx_timestamp > 100)) {
        core->rx_state = RS485_RX_IDLE;
        core->stats.error_count++;
    }
}

// 检查心跳
void RS485_Core_CheckHeartbeat(RS485_Core_t* core) {
    if (core == NULL) return;
    
    uint32_t current_time = HAL_GetTick();
    
    // 发送心跳
    if (current_time - core->last_heartbeat_tx > core->config.heartbeat_interval) {
        uint8_t heartbeat_data[4];
        heartbeat_data[0] = (uint8_t)(current_time & 0xFF);
        heartbeat_data[1] = (uint8_t)((current_time >> 8) & 0xFF);
        heartbeat_data[2] = (uint8_t)((current_time >> 16) & 0xFF);
        heartbeat_data[3] = (uint8_t)((current_time >> 24) & 0xFF);
        
        // 心跳命令码需要由协议层定义
        if (core->on_event) {
            core->on_event(0x02, current_time); // 心跳事件
        }
        
        core->last_heartbeat_tx = current_time;
    }
}

// 发送帧//@DMA发送
void RS485_Core_TransmitFrame(RS485_Core_t* core) {
    if (core == NULL) return;
    
    // 切换到发送模式
    RS485_Core_SetMode(core, true);
    
    // 发送数据
    HAL_StatusTypeDef status = HAL_UART_Transmit_IT(core->config.uart_handle, 
                                                     core->tx_buffer, 
                                                     core->tx_length);
    
    if (status == HAL_OK) {
        core->tx_state = RS485_TX_SENDING;
        core->tx_timestamp = HAL_GetTick();
        core->stats.tx_count++;
    } else {
        core->tx_state = RS485_TX_ERROR;
        RS485_Core_SetMode(core, false);
        if (core->on_error) {
            core->on_error(RS485_ERROR_UNKNOWN);
        }
    }
}

// UART发送完成回调（需要在HAL_UART_TxCpltCallback中调用）
void RS485_Core_TxCompleteCallback(RS485_Core_t* core) {
    if (core == NULL) return;
    
    // 切换回接收模式
    RS485_Core_SetMode(core, false);
    
    if (core->tx_state == RS485_TX_SENDING) {
        if (core->wait_ack) {
            core->tx_state = RS485_TX_WAIT_ACK;
        } else {
            core->tx_state = RS485_TX_COMPLETE;
        }
    }
}

// UART接收回调（需要在HAL_UART_RxCpltCallback中调用）
void RS485_Core_RxCallback(RS485_Core_t* core) {
    if (core == NULL) return;
    
    RS485_Core_ProcessRxByte(core, core->uart_rx_byte);
    
    // 继续接收
    HAL_UART_Receive_IT(core->config.uart_handle, &core->uart_rx_byte, 1);
}