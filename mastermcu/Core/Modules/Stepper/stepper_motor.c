#include "stepper_motor.h"
#include "../CAN/can.h"
#include <string.h>

/* 私有函数声明 */
static void StepperMotor_UpdateStatus(StepperMotor_t* motor);
static bool StepperMotor_WaitForResponse(uint32_t timeout);

/* 初始化步进电机 */
void StepperMotor_Init(StepperMotor_t* motor) {
    motor->node_id = STEPPER_NODE_ID;
    motor->control_word = 0;
    motor->status_word = 0;
    motor->current_position = 0;
    motor->target_position = 0;
    motor->last_response_time = HAL_GetTick();
    motor->is_connected = false;
    motor->is_enabled = false;
    motor->is_moving = false;
    
    /* 初始化CAN */
    CAN1_UserFilterStart();
    
    /* 延迟等待驱动器启动 */
    HAL_Delay(50);
    
    /* 设置电机使能选择 - 上电自动锁轴 */
    uint8_t enable_select = 0;  // 0=上电自动锁轴, 1=上电不锁轴
    StepperMotor_SendSDO(motor->node_id, OD_MOTOR_ENABLE_SELECT, 0, &enable_select, 1);
    
    /* 设置驱动器峰值电流 3000mA */
    uint16_t peak_current = 3000;
    StepperMotor_SendSDO(motor->node_id, OD_PEAK_CURRENT, 0, (uint8_t*)&peak_current, 2);
    
    /* 设置最小电流参数 - 保持电流为峰值的60% */
    uint16_t min_current = 0x3C3C;  // 高字节60=运行时60%, 低字节60=空闲时60%
    StepperMotor_SendSDO(motor->node_id, OD_MIN_CURRENT, 0, (uint8_t*)&min_current, 2);
    
    /* 设置细分数 4000脉冲/圈 */
    uint32_t subdivision = 4000;
    StepperMotor_SendSDO(motor->node_id, OD_SUBDIVISION, 0, (uint8_t*)&subdivision, 4);
    
    /* 设置操作模式为Profile Position */
    uint8_t mode = MODE_PROFILE_POSITION;
    StepperMotor_SendSDO(motor->node_id, OD_MODE_OF_OPERATION, 0, &mode, 1);
    
    /* 设置速度 */
    uint32_t velocity = STEPPER_SPEED;
    StepperMotor_SendSDO(motor->node_id, OD_PROFILE_VELOCITY, 0, (uint8_t*)&velocity, 4);
    
    /* 设置加减速度 */
    uint32_t acceleration = 10000;
    StepperMotor_SendSDO(motor->node_id, OD_PROFILE_ACCELERATION, 0, (uint8_t*)&acceleration, 4);
    StepperMotor_SendSDO(motor->node_id, OD_PROFILE_DECELERATION, 0, (uint8_t*)&acceleration, 4);
}

/* 使能电机 */
void StepperMotor_Enable(StepperMotor_t* motor) {
    /* 故障复位 */
    motor->control_word = 0x0080;  // 故障复位
    StepperMotor_SendSDO(motor->node_id, OD_CONTROL_WORD, 0, 
                        (uint8_t*)&motor->control_word, 2);
    
    /* 按照文档顺序：1.脱机 2.使能锁机 */
    motor->control_word = CW_SHUTDOWN;  // 0x0006 脱机
    StepperMotor_SendSDO(motor->node_id, OD_CONTROL_WORD, 0, 
                        (uint8_t*)&motor->control_word, 2);
    
    motor->control_word = CW_ENABLE;  // 0x0007 使能锁机
    StepperMotor_SendSDO(motor->node_id, OD_CONTROL_WORD, 0, 
                        (uint8_t*)&motor->control_word, 2);
    
    /* 再发送标准使能 */
    motor->control_word = CW_ENABLE_OPERATION;  // 0x000F
    StepperMotor_SendSDO(motor->node_id, OD_CONTROL_WORD, 0, 
                        (uint8_t*)&motor->control_word, 2);
    
    motor->is_enabled = true;
}

/* 禁用电机 */
void StepperMotor_Disable(StepperMotor_t* motor) {
    motor->control_word = CW_SHUTDOWN;  // 0x0006 脱机
    StepperMotor_SendSDO(motor->node_id, OD_CONTROL_WORD, 0, 
                        (uint8_t*)&motor->control_word, 2);
    motor->is_enabled = false;
    motor->is_moving = false;
}

/* 复位电机 */
void StepperMotor_Reset(StepperMotor_t* motor) {
    StepperMotor_Disable(motor);
    HAL_Delay(100);
    StepperMotor_Init(motor);
}

/* 绝对位置移动 */
void StepperMotor_MoveAbsolute(StepperMotor_t* motor, int32_t position) {
    if (!motor->is_enabled) {
        StepperMotor_Enable(motor);
    }
    
    /* 设置为绝对位置模式 */
    motor->control_word = CW_ENABLE_OPERATION;
    StepperMotor_SendSDO(motor->node_id, OD_CONTROL_WORD, 0, 
                        (uint8_t*)&motor->control_word, 2);
    
    /* 设置目标位置 */
    motor->target_position = position;
    StepperMotor_SendSDO(motor->node_id, OD_TARGET_POSITION, 0, 
                        (uint8_t*)&motor->target_position, 4);
    
    /* 启动运动 */
    motor->control_word = CW_ENABLE_OPERATION | CW_NEW_SETPOINT;
    StepperMotor_SendSDO(motor->node_id, OD_CONTROL_WORD, 0, 
                        (uint8_t*)&motor->control_word, 2);
    
    motor->control_word = CW_ENABLE_OPERATION;
    StepperMotor_SendSDO(motor->node_id, OD_CONTROL_WORD, 0, 
                        (uint8_t*)&motor->control_word, 2);
    
    motor->is_moving = true;
}

/* 相对位置移动 */
void StepperMotor_MoveRelative(StepperMotor_t* motor, int32_t steps) {
    if (!motor->is_enabled) {
        StepperMotor_Enable(motor);
    }
    
    /* C. 相对位置设置 - 先设置目标位置 */
    StepperMotor_SendSDO(motor->node_id, OD_TARGET_POSITION, 0, 
                        (uint8_t*)&steps, 4);
    
    /* 设置为相对位置模式 */
    motor->control_word = CW_ENABLE_OPERATION | CW_ABS_REL_BIT;  // 0x004F 相对模式
    StepperMotor_SendSDO(motor->node_id, OD_CONTROL_WORD, 0, 
                        (uint8_t*)&motor->control_word, 2);
    
    /* D. 相对位置即行 - 启动运动 */
    motor->control_word = CW_ENABLE_OPERATION | CW_ABS_REL_BIT | CW_NEW_SETPOINT | CW_CHANGE_IMMEDIATELY;  // 0x007F
    StepperMotor_SendSDO(motor->node_id, OD_CONTROL_WORD, 0, 
                        (uint8_t*)&motor->control_word, 2);
    
    motor->target_position = motor->current_position + steps;
    motor->is_moving = true;
}

/* 停止运动 */
void StepperMotor_Stop(StepperMotor_t* motor) {
    /* E. 减速停止 */
    motor->control_word = CW_HALT;  // 0x010F 停止
    StepperMotor_SendSDO(motor->node_id, OD_CONTROL_WORD, 0, 
                        (uint8_t*)&motor->control_word, 2);
    motor->is_moving = false;
}

/* 查询是否在运动 */
bool StepperMotor_IsMoving(StepperMotor_t* motor) {
    return motor->is_moving && !(motor->status_word & SW_TARGET_REACHED);
}

/* 查询是否使能 */
bool StepperMotor_IsEnabled(StepperMotor_t* motor) {
    return motor->is_enabled;
}

/* 查询是否连接 */
bool StepperMotor_IsConnected(StepperMotor_t* motor) {
    uint32_t current_time = HAL_GetTick();
    return (current_time - motor->last_response_time) < 1000;
}

/* 获取当前位置 */
int32_t StepperMotor_GetPosition(StepperMotor_t* motor) {
    return motor->current_position;
}

/* 获取状态字 */
uint16_t StepperMotor_GetStatus(StepperMotor_t* motor) {
    return motor->status_word;
}

/* 周期性更新 */
void StepperMotor_Update(StepperMotor_t* motor) {
    static uint32_t last_query_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    /* 每50ms查询一次状态 */
    if (current_time - last_query_time >= 50) {
        last_query_time = current_time;
        
        /* 读取状态字 */
        uint8_t data[4];
        uint8_t len = 2;
        if (StepperMotor_ReadSDO(motor->node_id, OD_STATUS_WORD, 0, data, &len)) {
            motor->status_word = data[0] | (data[1] << 8);
            motor->last_response_time = current_time;
            
            /* 检查是否到达目标 */
            if (motor->is_moving && (motor->status_word & SW_TARGET_REACHED)) {
                motor->is_moving = false;
            }
        }
        
        /* 读取当前位置 */
        len = 4;
        if (StepperMotor_ReadSDO(motor->node_id, OD_ACTUAL_POSITION, 0, data, &len)) {
            memcpy(&motor->current_position, data, 4);
        }
    }
    
    /* 更新连接状态 */
    motor->is_connected = StepperMotor_IsConnected(motor);
}

/* 发送SDO */
bool StepperMotor_SendSDO(uint8_t node_id, uint16_t index, uint8_t subindex, 
                          uint8_t* data, uint8_t len) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t tx_mailbox;
    
    /* 设置CAN帧头 */
    tx_header.StdId = 0x600 + node_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    
    /* SDO下载请求 */
    switch(len) {
        case 1:
            tx_data[0] = 0x2F;  /* 1字节 */
            break;
        case 2:
            tx_data[0] = 0x2B;  /* 2字节 */
            break;
        case 4:
            tx_data[0] = 0x23;  /* 4字节 */
            break;
        default:
            return false;
    }
    
    tx_data[1] = index & 0xFF;
    tx_data[2] = (index >> 8) & 0xFF;
    tx_data[3] = subindex;
    
    if (data && len > 0) {
        memcpy(&tx_data[4], data, len);
    }
    
    /* 发送CAN帧 */
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
        return false;
    }
    
    /* 简单延迟等待响应 */
    HAL_Delay(8);
    
    return true;
}

/* 读取SDO */
bool StepperMotor_ReadSDO(uint8_t node_id, uint16_t index, uint8_t subindex, 
                          uint8_t* data, uint8_t* len) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t tx_mailbox;
    
    /* 设置CAN帧头 */
    tx_header.StdId = 0x600 + node_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    
    /* SDO上传请求 */
    tx_data[0] = 0x40;
    tx_data[1] = index & 0xFF;
    tx_data[2] = (index >> 8) & 0xFF;
    tx_data[3] = subindex;
    
    /* 发送请求 */
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
        return false;
    }
    
    /* 等待响应 */
    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < 100) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            if (rx_header.StdId == (0x580 + node_id)) {
                /* 解析响应 */
                if ((rx_data[1] == (index & 0xFF)) && 
                    (rx_data[2] == ((index >> 8) & 0xFF)) &&
                    (rx_data[3] == subindex)) {
                    
                    /* 根据命令字节判断数据长度 */
                    switch(rx_data[0]) {
                        case 0x4F:  /* 1字节 */
                            *len = 1;
                            break;
                        case 0x4B:  /* 2字节 */
                            *len = 2;
                            break;
                        case 0x43:  /* 4字节 */
                            *len = 4;
                            break;
                        default:
                            return false;
                    }
                    
                    if (data) {
                        memcpy(data, &rx_data[4], *len);
                    }
                    return true;
                }
            }
        }
        HAL_Delay(1);
    }
    
    return false;
}
