#include "motor_fsm.h"
#include "can.h"
#include <stdio.h>
#include <string.h>

// 全局电机FSM实例
MotorFSM_t g_motor_fsm;

// 初始化电机FSM
void MotorFSM_Init(MotorFSM_t* fsm, MotorAdvanced_t* motor_adv) {
    memset(fsm, 0, sizeof(MotorFSM_t));
    
    fsm->motor_adv = motor_adv;
    fsm->current_state = MOTOR_STATE_IDLE;
    fsm->prev_state = MOTOR_STATE_IDLE;
    fsm->current_speed_percent = 0;
    fsm->target_speed_percent = 100;
}

// 主更新函数
void MotorFSM_Update(MotorFSM_t* fsm) {
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed = current_time - fsm->start_time;
    
    // 读取黑板数据
    bool motor_running = g_blackboard.motor_running;
    Direction_t dir = g_blackboard.dir;
    int motor_speed = g_blackboard.motor_speed;
    int position_offset = g_blackboard.position_offset;
    
    // 状态机处理
    switch (fsm->current_state) {
        case MOTOR_STATE_IDLE:
            if (motor_running) {
                // 启动电机
                fsm->target_position = Blackboard_FloorToPosition(g_blackboard.target_floor);
                fsm->start_position = fsm->motor_adv->actual_position;
                fsm->expected_pulses = abs(fsm->target_position - fsm->start_position);
                fsm->pulse_count = 0;
                fsm->start_time = current_time;
                fsm->current_state = MOTOR_STATE_ACCELERATING;
                
                // 生成脉冲
                MotorFSM_GeneratePulse(fsm, dir, motor_speed);
            } else if (position_offset != 0) {
                // 需要位置调整
                fsm->current_state = MOTOR_STATE_ADJUST;
                fsm->start_time = current_time;
            }
            break;
            
        case MOTOR_STATE_ACCELERATING:
            // 加速阶段
            if (!motor_running) {
                fsm->current_state = MOTOR_STATE_IDLE;
                break;
            }
            
            // 更新速度
            fsm->current_speed_percent += 10;  // 每次增加10%
            if (fsm->current_speed_percent >= motor_speed) {
                fsm->current_speed_percent = motor_speed;
                fsm->current_state = MOTOR_STATE_CRUISING;
            }
            
            // 继续生成脉冲
            MotorFSM_GeneratePulse(fsm, dir, fsm->current_speed_percent);
            break;
            
        case MOTOR_STATE_CRUISING:
            // 巡航阶段
            if (!motor_running) {
                // 停止命令
                fsm->current_state = MOTOR_STATE_IDLE;
                MotorAdv_Stop(fsm->motor_adv);
                Blackboard_PushEvent(EVENT_STOP_MOVING, g_blackboard.current_floor);
                break;
            }
            
            // 检查是否需要校准
            if (g_blackboard.sensors.sensor_triggered[g_blackboard.current_floor]) {
                fsm->current_state = MOTOR_STATE_WAIT_CALIBRATE;
                fsm->calibration_timeout = current_time + 500;
                break;
            }
            
            // 检查是否接近目标
            int32_t remaining = abs(fsm->target_position - fsm->motor_adv->actual_position);
            if (remaining < 1000) {  // 接近目标，开始减速
                fsm->current_speed_percent = 50;
            }
            
            // 继续移动
            MotorFSM_GeneratePulse(fsm, dir, fsm->current_speed_percent);
            
            // 检查超时
            if (elapsed > 30000) {  // 30秒超时
                fsm->current_state = MOTOR_STATE_TIMEOUT;
            }
            break;
            
        case MOTOR_STATE_WAIT_CALIBRATE:
            // 等待校准
            if (current_time > fsm->calibration_timeout) {
                // 校准完成，继续移动
                fsm->current_state = MOTOR_STATE_CRUISING;
                
                // 更新位置偏移
                int32_t expected_pos = Blackboard_FloorToPosition(g_blackboard.current_floor);
                int32_t actual_pos = fsm->motor_adv->actual_position;
                g_blackboard.position_offset = actual_pos - expected_pos;
                
                if (abs(g_blackboard.position_offset) > 100) {
                    Blackboard_PushEvent(EVENT_POSITION_ADJUST, g_blackboard.current_floor);
                }
            }
            break;
            
        case MOTOR_STATE_ADJUST:
            // 位置调整
            if (position_offset == 0) {
                fsm->current_state = MOTOR_STATE_IDLE;
                break;
            }
            
            // 计算调整步数
            int adjust_steps = MotorFSM_CalculateAdjustSteps(fsm, position_offset, dir);
            
            // 发送调整命令
            char cmd[64];
            snprintf(cmd, sizeof(cmd), "SetPos:%d,Speed:50", adjust_steps);
            MotorFSM_SendCANCommand(fsm, cmd);
            
            // 清除偏移
            g_blackboard.position_offset = 0;
            fsm->current_state = MOTOR_STATE_IDLE;
            break;
            
        case MOTOR_STATE_TIMEOUT:
            // 超时错误
            g_blackboard.error_code = 0x1001;  // 电机超时错误
            g_blackboard.motor_running = false;
            Blackboard_PushEvent(EVENT_ERROR, 0);
            fsm->current_state = MOTOR_STATE_ERROR;
            break;
            
        case MOTOR_STATE_ERROR:
            // 错误状态
            if (fsm->retry_count < 3 && elapsed > 5000) {
                // 尝试恢复
                fsm->retry_count++;
                fsm->current_state = MOTOR_STATE_IDLE;
                fsm->start_time = current_time;
            }
            break;
    }
}

// 生成脉冲
void MotorFSM_GeneratePulse(MotorFSM_t* fsm, Direction_t dir, int speed) {
    // 构建CAN命令
    char cmd[64];
    int steps = speed * 10;  // 速度转换为步数
    
    if (dir == DIR_UP) {
        snprintf(cmd, sizeof(cmd), "SetPos:%d,Speed:%d", steps, speed);
    } else {
        snprintf(cmd, sizeof(cmd), "SetPos:-%d,Speed:%d", steps, speed);
    }
    
    MotorFSM_SendCANCommand(fsm, cmd);
    
    // 更新脉冲计数
    fsm->pulse_count += steps;
}

// 计算调整步数
int MotorFSM_CalculateAdjustSteps(MotorFSM_t* fsm, int offset, Direction_t dir) {
    // 基于偏移计算需要的步数
    int steps = abs(offset);
    
    // 限制最大调整步数
    if (steps > 500) {
        steps = 500;
    }
    
    // 根据方向决定正负
    if ((offset > 0 && dir == DIR_DOWN) || (offset < 0 && dir == DIR_UP)) {
        steps = -steps;
    }
    
    return steps;
}

// 发送CAN命令
void MotorFSM_SendCANCommand(MotorFSM_t* fsm, const char* cmd) {
    // 将字符串命令转换为CAN数据
    uint8_t can_data[8] = {0};
    
    // 简化的命令编码（实际应根据CAN协议）
    if (strstr(cmd, "SetPos:") != NULL) {
        int pos, speed;
        if (sscanf(cmd, "SetPos:%d,Speed:%d", &pos, &speed) == 2) {
            can_data[0] = 0x02;  // 命令类型
            can_data[1] = 0x01;
            can_data[2] = 0x20;
            can_data[3] = pos & 0xFF;
            can_data[4] = (pos >> 8) & 0xFF;
            can_data[5] = 0x00;
            can_data[6] = speed & 0xFF;
            can_data[7] = (speed >> 8) & 0xFF;
            
            CAN1_Send_Num(fsm->motor_adv->motor->motor_can_id, can_data);
        }
    }
}

// 启动移动
void MotorFSM_StartMoving(MotorFSM_t* fsm, Direction_t dir, int speed_percent) {
    if (fsm->current_state == MOTOR_STATE_IDLE) {
        g_blackboard.motor_running = true;
        g_blackboard.dir = dir;
        g_blackboard.motor_speed = speed_percent;
    }
}

// 停止
void MotorFSM_Stop(MotorFSM_t* fsm) {
    g_blackboard.motor_running = false;
    MotorAdv_Stop(fsm->motor_adv);
    fsm->current_state = MOTOR_STATE_IDLE;
}

// 调整位置
void MotorFSM_AdjustPosition(MotorFSM_t* fsm, int offset) {
    g_blackboard.position_offset = offset;
}

// 查询是否空闲
bool MotorFSM_IsIdle(MotorFSM_t* fsm) {
    return fsm->current_state == MOTOR_STATE_IDLE;
}

// 查询是否有错误
bool MotorFSM_HasError(MotorFSM_t* fsm) {
    return fsm->current_state == MOTOR_STATE_ERROR;
}

// 获取位置
int MotorFSM_GetPosition(MotorFSM_t* fsm) {
    return fsm->motor_adv->actual_position;
}