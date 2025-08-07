#include "servo_control.h"
#include "rs485_slave.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

// 全局舵机控制实例
ServoControl_t g_servo_control;

// 初始化舵机控制模块
void ServoControl_Init(void) {
    // 清零结构体
    memset(&g_servo_control, 0, sizeof(ServoControl_t));
    
    // 设置初始状态
    g_servo_control.current_state = SERVO_STATE_IDLE;
    g_servo_control.prev_state = SERVO_STATE_IDLE;
    g_servo_control.current_pos = SERVO_MIN_ANGLE;  // 默认关门位置
    g_servo_control.target_pos = SERVO_MIN_ANGLE;
    g_servo_control.last_pos = SERVO_MIN_ANGLE;
    g_servo_control.state_entry_time = HAL_GetTick();
    
    // 初始化UART（假设使用UART3作为舵机通信）
    ServoControl_UART_Init();
    
    // 获取初始位置
    ServoControl_SendCommand(SERVO_CMD_GET_POSITION, NULL, 0);
}

// 主更新函数
void ServoControl_Update(void) {
    // 状态机处理
    switch (g_servo_control.current_state) {
        case SERVO_STATE_IDLE:
            ServoControl_HandleIdle();
            break;
            
        case SERVO_STATE_SENDING_CMD:
            ServoControl_HandleSendingCmd();
            break;
            
        case SERVO_STATE_WAIT_FEEDBACK:
            ServoControl_HandleWaitFeedback();
            break;
            
        case SERVO_STATE_ADJUST:
            ServoControl_HandleAdjust();
            break;
            
        case SERVO_STATE_TIMEOUT:
            ServoControl_HandleTimeout();
            break;
            
        case SERVO_STATE_ERROR:
            ServoControl_HandleError();
            break;
            
        default:
            g_servo_control.current_state = SERVO_STATE_IDLE;
            break;
    }
    
    // 更新状态进入时间
    if (g_servo_control.current_state != g_servo_control.prev_state) {
        g_servo_control.state_entry_time = HAL_GetTick();
        g_servo_control.prev_state = g_servo_control.current_state;
    }
    
    // 更新黑板舵机位置
    g_local_bb.servo_position = g_servo_control.current_pos;
    g_local_bb.servo_target_position = g_servo_control.target_pos;
    g_local_bb.servo_busy = (g_servo_control.current_state != SERVO_STATE_IDLE);
}

// 开门
void ServoControl_OpenDoor(void) {
    if (g_servo_control.current_state == SERVO_STATE_IDLE) {
        g_servo_control.target_pos = SERVO_MAX_ANGLE;
        g_servo_control.door_opening = true;
        g_servo_control.door_closing = false;
        g_servo_control.door_action_start = HAL_GetTick();
        g_servo_control.retry_count = 0;
        
        // 更新黑板门状态
        LocalBlackboard_SetDoorState(DOOR_OPENING);
        
        // 设置位置
        ServoControl_SetPosition(SERVO_MAX_ANGLE);
    }
}

// 关门
void ServoControl_CloseDoor(void) {
    if (g_servo_control.current_state == SERVO_STATE_IDLE || 
        g_servo_control.current_state == SERVO_STATE_ERROR) {
        g_servo_control.target_pos = SERVO_MIN_ANGLE;
        g_servo_control.door_opening = false;
        g_servo_control.door_closing = true;
        g_servo_control.door_action_start = HAL_GetTick();
        g_servo_control.retry_count = 0;
        
        // 更新黑板门状态
        LocalBlackboard_SetDoorState(DOOR_CLOSING);
        
        // 设置位置
        ServoControl_SetPosition(SERVO_MIN_ANGLE);
    }
}

// 停止门
void ServoControl_StopDoor(void) {
    ServoControl_SendCommand(SERVO_CMD_STOP, NULL, 0);
    g_servo_control.door_opening = false;
    g_servo_control.door_closing = false;
    g_servo_control.current_state = SERVO_STATE_IDLE;
}

// 检查门是否开启
bool ServoControl_IsDoorOpen(void) {
    return (g_servo_control.current_pos >= SERVO_MAX_ANGLE - SERVO_POSITION_TOLERANCE);
}

// 检查门是否关闭
bool ServoControl_IsDoorClosed(void) {
    return (g_servo_control.current_pos <= SERVO_MIN_ANGLE + SERVO_POSITION_TOLERANCE);
}

// 检查门是否在移动
bool ServoControl_IsDoorMoving(void) {
    return g_servo_control.door_opening || g_servo_control.door_closing || 
           (g_servo_control.current_state != SERVO_STATE_IDLE && 
            g_servo_control.current_state != SERVO_STATE_ERROR);
}

// 设置位置
void ServoControl_SetPosition(int position) {
    if (position < SERVO_MIN_ANGLE) position = SERVO_MIN_ANGLE;
    if (position > SERVO_MAX_ANGLE) position = SERVO_MAX_ANGLE;
    
    g_servo_control.target_pos = position;
    
    // 构建参数
    uint8_t params[2];
    params[0] = (uint8_t)(position & 0xFF);
    params[1] = (uint8_t)((position >> 8) & 0xFF);
    
    // 发送命令
    if (ServoControl_SendCommand(SERVO_CMD_SET_POSITION, params, 2)) {
        g_servo_control.current_state = SERVO_STATE_SENDING_CMD;
        g_servo_control.start_time = HAL_GetTick();
    }
}

// 获取位置
int ServoControl_GetPosition(void) {
    return g_servo_control.current_pos;
}

// 检查是否在目标位置
bool ServoControl_IsAtPosition(int position) {
    return abs(g_servo_control.current_pos - position) <= SERVO_POSITION_TOLERANCE;
}

// 发送命令
bool ServoControl_SendCommand(uint8_t cmd, uint8_t* params, uint8_t param_len) {
    if (g_servo_control.waiting_response) {
        return false;
    }
    
    // 构建命令帧
    g_servo_control.tx_buffer[0] = 0xFF;  // 起始标志
    g_servo_control.tx_buffer[1] = 0xFF;  // 起始标志
    g_servo_control.tx_buffer[2] = 0x01;  // 舵机ID
    g_servo_control.tx_buffer[3] = param_len + 2;  // 长度
    g_servo_control.tx_buffer[4] = cmd;    // 命令
    
    // 复制参数
    if (param_len > 0 && params != NULL) {
        memcpy(&g_servo_control.tx_buffer[5], params, param_len);
    }
    
    // 计算校验和
    uint8_t checksum = 0;
    for (int i = 2; i < 5 + param_len; i++) {
        checksum += g_servo_control.tx_buffer[i];
    }
    g_servo_control.tx_buffer[5 + param_len] = ~checksum;
    
    // 发送数据
    for (int i = 0; i < 6 + param_len; i++) {
        ServoControl_UART_SendByte(g_servo_control.tx_buffer[i]);
    }
    
    g_servo_control.last_cmd = cmd;
    g_servo_control.waiting_response = true;
    g_servo_control.rx_index = 0;
    
    return true;
}

// 读取反馈
bool ServoControl_ReadFeedback(ServoFeedback_t* feedback) {
    if (!g_servo_control.has_feedback) {
        return false;
    }
    
    // 解析反馈数据
    if (g_servo_control.rx_index >= 8) {
        feedback->status = g_servo_control.rx_buffer[5];
        feedback->position = (int16_t)((g_servo_control.rx_buffer[6] << 8) | g_servo_control.rx_buffer[7]);
        feedback->error_flags = g_servo_control.rx_buffer[4];
        
        g_servo_control.has_feedback = false;
        return true;
    }
    
    return false;
}

// 计算调整量
int ServoControl_CalculateAdjust(int error) {
    // 简单的比例调整
    if (abs(error) <= SERVO_POSITION_TOLERANCE) {
        return 0;
    }
    
    int adjust = error / 2;
    if (adjust == 0) {
        adjust = (error > 0) ? 1 : -1;
    }
    
    return adjust;
}

// 处理反馈
void ServoControl_ProcessFeedback(void) {
    ServoFeedback_t feedback;
    
    if (ServoControl_ReadFeedback(&feedback)) {
        // 更新当前位置
        g_servo_control.current_pos = feedback.position;
        
        // 检查错误
        if (feedback.error_flags != 0) {
            g_servo_control.error_code = feedback.error_flags;
            g_servo_control.error_count++;
            g_servo_control.current_state = SERVO_STATE_ERROR;
            return;
        }
        
        // 检查是否到达目标位置
        if (ServoControl_IsAtPosition(g_servo_control.target_pos)) {
            // 更新门状态
            if (g_servo_control.door_opening && ServoControl_IsDoorOpen()) {
                LocalBlackboard_SetDoorState(DOOR_OPEN);
                g_servo_control.door_opening = false;
            } else if (g_servo_control.door_closing && ServoControl_IsDoorClosed()) {
                LocalBlackboard_SetDoorState(DOOR_CLOSED);
                g_servo_control.door_closing = false;
            }
            
            g_servo_control.current_state = SERVO_STATE_IDLE;
        } else {
            // 需要调整
            g_servo_control.current_state = SERVO_STATE_ADJUST;
        }
    }
}

// UART初始化（使用UART3）
void ServoControl_UART_Init(void) {
    // UART初始化应该在HAL层完成
    // 这里只是设置相关参数
    g_servo_control.rx_index = 0;
    g_servo_control.waiting_response = false;
    g_servo_control.has_feedback = false;
}

// UART发送字节
void ServoControl_UART_SendByte(uint8_t data) {
    HAL_UART_Transmit(&huart3, &data, 1, 10);
}

// UART接收回调
void ServoControl_UART_RxCallback(uint8_t data) {
    if (g_servo_control.rx_index < sizeof(g_servo_control.rx_buffer)) {
        g_servo_control.rx_buffer[g_servo_control.rx_index++] = data;
        
        // 检查是否接收完成
        if (g_servo_control.rx_index >= 2) {
            if (g_servo_control.rx_buffer[0] == 0xFF && g_servo_control.rx_buffer[1] == 0xFF) {
                // 检查长度字段
                if (g_servo_control.rx_index >= 4) {
                    uint8_t expected_len = g_servo_control.rx_buffer[3] + 4;
                    if (g_servo_control.rx_index >= expected_len) {
                        // 接收完成
                        g_servo_control.has_feedback = true;
                        g_servo_control.waiting_response = false;
                    }
                }
            } else {
                // 无效起始标志，重置
                g_servo_control.rx_index = 0;
            }
        }
    }
}

// 等待响应
bool ServoControl_WaitResponse(uint32_t timeout) {
    uint32_t start = HAL_GetTick();
    
    while (g_servo_control.waiting_response) {
        if (HAL_GetTick() - start > timeout) {
            return false;
        }
        HAL_Delay(1);
    }
    
    return g_servo_control.has_feedback;
}

// 获取状态
ServoState_t ServoControl_GetState(void) {
    return g_servo_control.current_state;
}

// 检查是否有错误
bool ServoControl_HasError(void) {
    return (g_servo_control.current_state == SERVO_STATE_ERROR) || 
           (g_servo_control.error_count > 0);
}

// 获取错误代码
uint16_t ServoControl_GetErrorCode(void) {
    return g_servo_control.error_code;
}

// 打印状态
void ServoControl_PrintStatus(void) {
    printf("=== Servo Control Status ===\n");
    printf("State: %d\n", g_servo_control.current_state);
    printf("Current Position: %d°\n", g_servo_control.current_pos);
    printf("Target Position: %d°\n", g_servo_control.target_pos);
    printf("Door Opening: %s\n", g_servo_control.door_opening ? "Yes" : "No");
    printf("Door Closing: %s\n", g_servo_control.door_closing ? "Yes" : "No");
    printf("Error Count: %d\n", g_servo_control.error_count);
    printf("Error Code: 0x%04X\n", g_servo_control.error_code);
    printf("===========================\n");
}

// 状态处理函数 - 空闲
void ServoControl_HandleIdle(void) {
    // 检查是否有新的位置命令
    if (abs(g_servo_control.target_pos - g_servo_control.current_pos) > SERVO_POSITION_TOLERANCE) {
        ServoControl_SetPosition(g_servo_control.target_pos);
    }
    
    // 定期获取位置反馈
    static uint32_t last_query_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - last_query_time > 500) {  // 每500ms查询一次
        last_query_time = current_time;
        ServoControl_SendCommand(SERVO_CMD_GET_POSITION, NULL, 0);
    }
}

// 状态处理函数 - 发送命令
void ServoControl_HandleSendingCmd(void) {
    // 立即转换到等待反馈状态
    g_servo_control.current_state = SERVO_STATE_WAIT_FEEDBACK;
    g_servo_control.start_time = HAL_GetTick();
}

// 状态处理函数 - 等待反馈
void ServoControl_HandleWaitFeedback(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 检查超时
    if (current_time - g_servo_control.start_time > SERVO_TIMEOUT) {
        g_servo_control.current_state = SERVO_STATE_TIMEOUT;
        return;
    }
    
    // 处理反馈
    if (g_servo_control.has_feedback) {
        ServoControl_ProcessFeedback();
    }
}

// 状态处理函数 - 调整
void ServoControl_HandleAdjust(void) {
    // 计算位置误差
    int error = g_servo_control.target_pos - g_servo_control.current_pos;
    
    if (abs(error) <= SERVO_POSITION_TOLERANCE) {
        // 到达目标位置
        g_servo_control.current_state = SERVO_STATE_IDLE;
        
        // 更新门状态
        if (g_servo_control.door_opening && ServoControl_IsDoorOpen()) {
            LocalBlackboard_SetDoorState(DOOR_OPEN);
            g_servo_control.door_opening = false;
        } else if (g_servo_control.door_closing && ServoControl_IsDoorClosed()) {
            LocalBlackboard_SetDoorState(DOOR_CLOSED);
            g_servo_control.door_closing = false;
        }
    } else {
        // 需要继续调整
        int adjust = ServoControl_CalculateAdjust(error);
        ServoControl_SetPosition(g_servo_control.current_pos + adjust);
    }
}

// 状态处理函数 - 超时
void ServoControl_HandleTimeout(void) {
    g_servo_control.retry_count++;
    g_servo_control.waiting_response = false;
    
    if (g_servo_control.retry_count >= SERVO_RETRY_MAX) {
        g_servo_control.current_state = SERVO_STATE_ERROR;
        g_servo_control.error_code = 0x301;  // 舵机通信超时
        
        // 更新黑板
        g_local_bb.error_code = 0x301;
        LocalBlackboard_MarkForSync(SYNC_FIELD_ERROR);
        LocalBlackboard_SetDoorState(DOOR_ERROR);
        
        // 发送错误报告
        RS485_Slave_SendError(0x301);
    } else {
        // 重试
        g_servo_control.current_state = SERVO_STATE_IDLE;
        HAL_Delay(100);  // 短暂延时后重试
    }
}

// 状态处理函数 - 错误
void ServoControl_HandleError(void) {
    static uint32_t error_report_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 定期报告错误
    if (current_time - error_report_time > 5000) {
        error_report_time = current_time;
        
        // 发送错误状态
        RS485_Slave_SendDoorStatus(DOOR_ERROR);
        
        // 尝试恢复
        g_servo_control.error_count = 0;
        g_servo_control.current_state = SERVO_STATE_IDLE;
        
        // 查询当前位置
        ServoControl_SendCommand(SERVO_CMD_GET_POSITION, NULL, 0);
    }
}