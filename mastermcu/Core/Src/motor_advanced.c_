#include "motor_advanced.h"
#include "blackboard.h"
#include <stdlib.h>
#include <math.h>

// 初始化高级电机控制
void MotorAdv_Init(MotorAdvanced_t* motor_adv, Motor_t* motor) {
    motor_adv->motor = motor;
    
    // 默认速度曲线参数
    motor_adv->profile.start_speed = 200;
    motor_adv->profile.cruise_speed = 1500;
    motor_adv->profile.end_speed = 200;
    motor_adv->profile.accel_rate = 100;
    motor_adv->profile.decel_rate = 100;
    
    // 初始状态
    motor_adv->stage = SPEED_STAGE_IDLE;
    motor_adv->cmd_position = 0;
    motor_adv->actual_position = 0;
    motor_adv->position_error = 0;
    motor_adv->current_speed = 0;
    motor_adv->is_calibrated = false;
    motor_adv->is_homing = false;
    motor_adv->home_offset = 0;
}

// 计算速度曲线
void MotorAdv_CalculateProfile(MotorAdvanced_t* motor_adv, int32_t target_position) {
    int32_t total_distance = abs(target_position - motor_adv->actual_position);
    
    // 计算加速距离（使用运动学公式）
    // d = (v²-v0²)/(2a)
    uint32_t accel_distance = ((motor_adv->profile.cruise_speed * motor_adv->profile.cruise_speed) - 
                              (motor_adv->profile.start_speed * motor_adv->profile.start_speed)) / 
                              (2 * motor_adv->profile.accel_rate);
    
    uint32_t decel_distance = ((motor_adv->profile.cruise_speed * motor_adv->profile.cruise_speed) - 
                              (motor_adv->profile.end_speed * motor_adv->profile.end_speed)) / 
                              (2 * motor_adv->profile.decel_rate);
    
    // 检查是否有足够距离达到巡航速度
    if (accel_distance + decel_distance >= total_distance) {
        // 三角形速度曲线（没有匀速段）
        motor_adv->profile.accel_steps = total_distance / 2;
        motor_adv->profile.decel_steps = total_distance - motor_adv->profile.accel_steps;
        motor_adv->profile.cruise_steps = 0;
        
        // 重新计算最大速度
        uint16_t max_speed = sqrt(motor_adv->profile.start_speed * motor_adv->profile.start_speed + 
                                 2 * motor_adv->profile.accel_rate * motor_adv->profile.accel_steps);
        if (max_speed < motor_adv->profile.cruise_speed) {
            motor_adv->profile.cruise_speed = max_speed;
        }
    } else {
        // 梯形速度曲线
        motor_adv->profile.accel_steps = accel_distance;
        motor_adv->profile.decel_steps = decel_distance;
        motor_adv->profile.cruise_steps = total_distance - accel_distance - decel_distance;
    }
    
    motor_adv->total_steps = total_distance;
    motor_adv->stage_steps = 0;
}

// 更新速度
void MotorAdv_UpdateSpeed(MotorAdvanced_t* motor_adv) {
    if (motor_adv->stage == SPEED_STAGE_IDLE) {
        return;
    }
    
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - motor_adv->last_update_time;
    
    if (elapsed_time < 10) {  // 10ms更新周期
        return;
    }
    
    motor_adv->last_update_time = current_time;
    
    switch (motor_adv->stage) {
        case SPEED_STAGE_ACCEL:
            motor_adv->current_speed += motor_adv->profile.accel_rate * elapsed_time / 1000;
            if (motor_adv->current_speed >= motor_adv->profile.cruise_speed) {
                motor_adv->current_speed = motor_adv->profile.cruise_speed;
                motor_adv->stage = (motor_adv->profile.cruise_steps > 0) ? 
                                  SPEED_STAGE_CRUISE : SPEED_STAGE_DECEL;
                motor_adv->stage_steps = 0;
            }
            break;
            
        case SPEED_STAGE_CRUISE:
            // 保持匀速
            if (motor_adv->stage_steps >= motor_adv->profile.cruise_steps) {
                motor_adv->stage = SPEED_STAGE_DECEL;
                motor_adv->stage_steps = 0;
            }
            break;
            
        case SPEED_STAGE_DECEL:
            motor_adv->current_speed -= motor_adv->profile.decel_rate * elapsed_time / 1000;
            if (motor_adv->current_speed <= motor_adv->profile.end_speed) {
                motor_adv->current_speed = motor_adv->profile.end_speed;
            }
            break;
            
        default:
            break;
    }
    
    // 更新电机速度
    Motor_SetSpeed(motor_adv->motor, motor_adv->current_speed);
}

// 移动到指定位置
void MotorAdv_MoveToPosition(MotorAdvanced_t* motor_adv, int32_t position) {
    motor_adv->cmd_position = position;
    
    // 计算速度曲线
    MotorAdv_CalculateProfile(motor_adv, position);
    
    // 开始运动
    motor_adv->stage = SPEED_STAGE_ACCEL;
    motor_adv->stage_steps = 0;
    motor_adv->move_start_time = HAL_GetTick();
    motor_adv->last_update_time = motor_adv->move_start_time;
    motor_adv->current_speed = motor_adv->profile.start_speed;
    
    // 设置电机目标位置
    Motor_MoveTo(motor_adv->motor, position);
}

// 相对移动
void MotorAdv_MoveRelative(MotorAdvanced_t* motor_adv, int32_t steps) {
    int32_t target = motor_adv->actual_position + steps;
    MotorAdv_MoveToPosition(motor_adv, target);
}

// 停止
void MotorAdv_Stop(MotorAdvanced_t* motor_adv) {
    motor_adv->stage = SPEED_STAGE_DECEL;
    motor_adv->profile.end_speed = 0;
    motor_adv->profile.decel_rate = 200;  // 快速减速
}

// 紧急停止
void MotorAdv_EmergencyStop(MotorAdvanced_t* motor_adv) {
    Motor_Stop(motor_adv->motor);
    motor_adv->stage = SPEED_STAGE_IDLE;
    motor_adv->current_speed = 0;
    motor_adv->cmd_position = motor_adv->actual_position;
}

// 开始归位
void MotorAdv_StartHoming(MotorAdvanced_t* motor_adv) {
    motor_adv->is_homing = true;
    motor_adv->is_calibrated = false;
    
    // 慢速向下移动寻找原点
    motor_adv->profile.cruise_speed = 300;  // 归位速度慢
    motor_adv->profile.accel_rate = 50;
    motor_adv->profile.decel_rate = 50;
    
    // 向下移动足够距离
    MotorAdv_MoveRelative(motor_adv, -100000);
}

// 设置当前位置为原点
void MotorAdv_SetHomePosition(MotorAdvanced_t* motor_adv) {
    motor_adv->actual_position = 0;
    motor_adv->cmd_position = 0;
    motor_adv->home_offset = 0;
    motor_adv->is_calibrated = true;
    motor_adv->is_homing = false;
    
    // 停止电机
    MotorAdv_EmergencyStop(motor_adv);
    
    // 恢复正常速度参数
    motor_adv->profile.cruise_speed = g_blackboard.motor_cruise_speed;
    motor_adv->profile.accel_rate = g_blackboard.motor_accel_rate;
    motor_adv->profile.decel_rate = g_blackboard.motor_accel_rate;
}

// 更新位置（从传感器）
void MotorAdv_UpdatePosition(MotorAdvanced_t* motor_adv, int32_t sensor_position) {
    // 计算误差
    motor_adv->position_error = sensor_position - motor_adv->actual_position;
    
    // 如果误差太大，进行校正
    if (abs(motor_adv->position_error) > 100) {  // 100脉冲容差
        MotorAdv_CorrectPosition(motor_adv, sensor_position);
    }
}

// 校正位置
void MotorAdv_CorrectPosition(MotorAdvanced_t* motor_adv, int32_t actual_position) {
    motor_adv->actual_position = actual_position;
    motor_adv->position_error = 0;
    
    // 更新黑板数据
    Blackboard_UpdateMotorPosition(actual_position);
}

// 检查是否到达目标
bool MotorAdv_IsAtTarget(MotorAdvanced_t* motor_adv) {
    return abs(motor_adv->cmd_position - motor_adv->actual_position) < 10 &&  // 10脉冲容差
           motor_adv->stage == SPEED_STAGE_IDLE;
}

// 获取当前位置
int32_t MotorAdv_GetPosition(MotorAdvanced_t* motor_adv) {
    return motor_adv->actual_position;
}

// 获取当前速度
uint16_t MotorAdv_GetSpeed(MotorAdvanced_t* motor_adv) {
    return motor_adv->current_speed;
}

// 获取当前阶段
SpeedStage_t MotorAdv_GetStage(MotorAdvanced_t* motor_adv) {
    return motor_adv->stage;
}

// 是否在归位
bool MotorAdv_IsHoming(MotorAdvanced_t* motor_adv) {
    return motor_adv->is_homing;
}

// 是否已校准
bool MotorAdv_IsCalibrated(MotorAdvanced_t* motor_adv) {
    return motor_adv->is_calibrated;
}

// 主处理循环
void MotorAdv_Process(MotorAdvanced_t* motor_adv) {
    // 更新速度曲线
    MotorAdv_UpdateSpeed(motor_adv);
    
    // 更新位置（简化模拟，实际应从编码器读取）
    if (motor_adv->motor->is_moving) {
        uint32_t elapsed = HAL_GetTick() - motor_adv->last_update_time;
        int32_t distance = (motor_adv->current_speed * elapsed) / 1000;
        
        if (motor_adv->cmd_position > motor_adv->actual_position) {
            motor_adv->actual_position += distance;
            if (motor_adv->actual_position > motor_adv->cmd_position) {
                motor_adv->actual_position = motor_adv->cmd_position;
            }
        } else {
            motor_adv->actual_position -= distance;
            if (motor_adv->actual_position < motor_adv->cmd_position) {
                motor_adv->actual_position = motor_adv->cmd_position;
            }
        }
        
        motor_adv->stage_steps += distance;
        
        // 检查是否到达目标
        if (motor_adv->actual_position == motor_adv->cmd_position) {
            motor_adv->stage = SPEED_STAGE_IDLE;
            motor_adv->current_speed = 0;
            motor_adv->motor->is_moving = false;
            Motor_Stop(motor_adv->motor);
        }
    }
    
    // 更新黑板
    g_blackboard.motor.current_position = motor_adv->actual_position;
    g_blackboard.motor.current_speed = motor_adv->current_speed;
    g_blackboard.motor.is_moving = motor_adv->motor->is_moving;
}