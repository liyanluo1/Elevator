#include "motor_control.h"

// 初始化电机
void Motor_Init(Motor_t* motor, uint16_t motor_id) {
    motor->motor_can_id = motor_id;
    motor->current_position = 0;
    motor->target_position = 0;
    motor->is_moving = false;
    motor->speed = 1000;  // 默认速度
    motor->status = MOTOR_STATUS_IDLE;
    motor->last_command_time = 0;
    motor->last_response_time = HAL_GetTick();

    // 启动CAN（如果还未启动）
    CAN1_UserFilterStart();
}

// 使用厂商协议发送基本运动命令
uint8_t Motor_SendBasicCommand(Motor_t* motor, uint8_t cmd_type) {
    uint8_t data[8] = {0x02, 0x01, 0x20, 0x03, 0x84, 0x00, 0x64, 0x00};  // 8字节，补零

    // 根据命令类型修改参数
    switch(cmd_type) {
        case 1:  // 基本运动命令1
            data[3] = 0x03;
            data[4] = 0x84;
            break;
        case 2:  // 基本运动命令2
            data[3] = 0x0E;
            data[4] = 0x10;
            break;
        case 3:  // 基本运动命令3
            data[3] = 0x07;
            data[4] = 0x08;
            break;
        default:
            return 1; // 无效命令
    }

    motor->is_moving = true;
    motor->last_command_time = HAL_GetTick();

    return CAN1_Send_Num(motor->motor_can_id, data);
}

// 绝对位置移动（需要根据厂商文档调整）
uint8_t Motor_MoveTo(Motor_t* motor, int32_t position) {
    uint8_t data[8] = {0x02, 0x01, 0x20, 0x00, 0x00, 0x00, 0x64, 0x00};

    // 将位置值编码到数据中（根据厂商协议调整）
    data[3] = position & 0xFF;        // 位置低字节
    data[4] = (position >> 8) & 0xFF; // 位置高字节

    motor->target_position = position;
    motor->is_moving = true;
    motor->last_command_time = HAL_GetTick();

    return CAN1_Send_Num(motor->motor_can_id, data);
}

// 相对位置移动
uint8_t Motor_MoveSteps(Motor_t* motor, int32_t steps) {
    // 使用厂商的基本命令之一
    if (steps > 0) {
        return Motor_SendBasicCommand(motor, 1);  // 正向运动
    } else if (steps < 0) {
        return Motor_SendBasicCommand(motor, 2);  // 反向运动
    } else {
        return Motor_Stop(motor);  // 停止
    }
}

// 设置速度（需要根据厂商协议调整）
uint8_t Motor_SetSpeed(Motor_t* motor, uint16_t speed) {
    uint8_t data[8] = {0x02, 0x01, 0x20, 0x03, 0x84, 0x00, 0x00, 0x00};

    // 将速度值编码到最后两个字节
    data[5] = speed & 0xFF;        // 速度低字节
    data[6] = (speed >> 8) & 0xFF; // 速度高字节

    motor->speed = speed;
    motor->last_command_time = HAL_GetTick();

    return CAN1_Send_Num(motor->motor_can_id, data);
}

// 停止电机
uint8_t Motor_Stop(Motor_t* motor) {
    uint8_t data[8] = {0x02, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};  // 全零命令

    motor->is_moving = false;
    motor->target_position = motor->current_position;
    motor->last_command_time = HAL_GetTick();

    return CAN1_Send_Num(motor->motor_can_id, data);
}

// 设置零点
uint8_t Motor_SetZero(Motor_t* motor) {
    motor->current_position = 0;
    motor->target_position = 0;
    motor->last_command_time = HAL_GetTick();

    // 发送复位命令（如果厂商支持）
    uint8_t data[8] = {0x02, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
    return CAN1_Send_Num(motor->motor_can_id, data);
}

// 读取位置（厂商可能不支持，模拟实现）
uint8_t Motor_ReadPosition(Motor_t* motor) {
    motor->last_command_time = HAL_GetTick();
    // 厂商协议可能不支持位置读取，这里只是更新时间戳
    return 0;
}

// 读取状态（厂商可能不支持，模拟实现）
uint8_t Motor_ReadStatus(Motor_t* motor) {
    motor->last_command_time = HAL_GetTick();
    // 厂商协议可能不支持状态读取，这里只是更新时间戳
    return 0;
}

// 处理CAN接收消息（厂商可能不发送响应）
void Motor_ProcessRxMessage(Motor_t* motor, uint16_t rx_id, uint8_t* data) {
    // 检查是否是该电机的响应
    if (rx_id != motor->motor_can_id) {
        return;
    }

    motor->last_response_time = HAL_GetTick();

    // 根据厂商协议解析响应（如果有的话）
    // 这里可能需要根据实际情况调整
}

// 获取当前位置
int32_t Motor_GetPosition(Motor_t* motor) {
    return motor->current_position;
}

// 获取目标位置
int32_t Motor_GetTargetPosition(Motor_t* motor) {
    return motor->target_position;
}

// 检查是否正在运动
bool Motor_IsMoving(Motor_t* motor) {
    return motor->is_moving;
}

// 获取状态
uint8_t Motor_GetStatus(Motor_t* motor) {
    return motor->status;
}

// 获取速度
uint16_t Motor_GetSpeed(Motor_t* motor) {
    return motor->speed;
}

// 检查连接状态（简化版本，因为厂商协议可能不提供反馈）
bool Motor_IsConnected(Motor_t* motor) {
    // 由于厂商协议可能不提供反馈，这里简单返回true
    // 或者可以基于命令发送的成功与否判断
    return true;
}

// 电机任务处理（简化版本）
void Motor_Task(Motor_t* motor) {
    static uint32_t last_check = 0;
    uint32_t current_time = HAL_GetTick();

    // 每2秒检查一次运动状态
    if (current_time - last_check >= 2000) {
        // 如果运动时间超过5秒，认为已经停止
        if (motor->is_moving && (current_time - motor->last_command_time > 5000)) {
            motor->is_moving = false;
            motor->status = MOTOR_STATUS_IDLE;
        }
        last_check = current_time;
    }
}

// 厂商示例运动函数
uint8_t Motor_RunExample1(Motor_t* motor) {
    return Motor_SendBasicCommand(motor, 1);  // 使用厂商的CAN_SENT1命令
}

uint8_t Motor_RunExample2(Motor_t* motor) {
    return Motor_SendBasicCommand(motor, 2);  // 使用厂商的CAN_SENT2命令
}

uint8_t Motor_RunExample3(Motor_t* motor) {
    return Motor_SendBasicCommand(motor, 3);  // 使用厂商的CAN_SENT3命令
}
