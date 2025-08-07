#include "servo_control.h"
#include "servo_fsm.h"
#include <stdio.h>
#include <math.h>
#include "stm32f1xx_hal.h"

// PID参数（默认值）
static PIDParams_t pid_params = {
    .kp = 2.0f,           // 比例系数
    .ki = 0.1f,           // 积分系数
    .kd = 0.5f,           // 微分系数
    .max_output = 3000,   // 最大速度
    .min_output = 100     // 最小速度
};

/**
 * @brief 初始化舵机控制
 */
void ServoControl_Init(ServoControl_t *ctrl, uint8_t id) {
    ctrl->id = id;
    ctrl->target_position = 0;
    ctrl->current_position = 0;
    ctrl->position_error = 0;
    ctrl->last_error = 0;
    ctrl->error_sum = 0;
    ctrl->speed = 1000;  // 默认速度
    ctrl->move_start_time = 0;
    ctrl->is_enabled = 0;
    
    // 初始化状态机
    ServoFsm_Init(&ctrl->fsm);
    
    // 读取当前位置
    ctrl->current_position = servo_get_position(id);
    ctrl->target_position = ctrl->current_position;
    
    printf("[SERVO_CTRL] Initialized ID=%d, Position=%d\r\n", 
           id, ctrl->current_position);
}

/**
 * @brief 设置PID参数
 */
void ServoControl_SetPID(PIDParams_t *pid) {
    pid_params = *pid;
    printf("[SERVO_CTRL] PID updated: Kp=%d.%02d, Ki=%d.%02d, Kd=%d.%02d\r\n",
           (int)pid_params.kp, (int)(pid_params.kp * 100) % 100,
           (int)pid_params.ki, (int)(pid_params.ki * 100) % 100,
           (int)pid_params.kd, (int)(pid_params.kd * 100) % 100);
}

/**
 * @brief 使能舵机控制
 */
void ServoControl_Enable(ServoControl_t *ctrl) {
    servo_set_torque_enable(ctrl->id, 1);
    ctrl->is_enabled = 1;
    ctrl->error_sum = 0;  // 清除积分
    printf("[SERVO_CTRL] Enabled ID=%d\r\n", ctrl->id);
}

/**
 * @brief 禁用舵机控制
 */
void ServoControl_Disable(ServoControl_t *ctrl) {
    servo_set_torque_enable(ctrl->id, 0);
    ctrl->is_enabled = 0;
    ServoFsm_Reset(&ctrl->fsm);
    printf("[SERVO_CTRL] Disabled ID=%d\r\n", ctrl->id);
}

/**
 * @brief 设置目标位置
 */
void ServoControl_SetTargetPosition(ServoControl_t *ctrl, int16_t position) {
    ctrl->target_position = position;
    ctrl->move_start_time = HAL_GetTick();
    ctrl->error_sum = 0;  // 清除积分
    
    // 触发状态机开始移动事件
    ServoFsm_ProcessEvent(&ctrl->fsm, SERVO_EVENT_START_MOVE);
    
    printf("[SERVO_CTRL] Target set: %d (current: %d)\r\n", 
           position, ctrl->current_position);
}

/**
 * @brief 设置速度
 */
void ServoControl_SetSpeed(ServoControl_t *ctrl, uint16_t speed) {
    ctrl->speed = speed;
    servo_set_speed(ctrl->id, speed);
}

/**
 * @brief PID控制计算
 */
static int16_t calculate_pid_output(ServoControl_t *ctrl) {
    // 计算误差
    ctrl->position_error = ctrl->target_position - ctrl->current_position;
    
    // P项
    float p_term = pid_params.kp * ctrl->position_error;
    
    // I项（积分饱和限制）
    ctrl->error_sum += ctrl->position_error;
    if (ctrl->error_sum > 1000) ctrl->error_sum = 1000;
    if (ctrl->error_sum < -1000) ctrl->error_sum = -1000;
    float i_term = pid_params.ki * ctrl->error_sum;
    
    // D项
    float d_term = pid_params.kd * (ctrl->position_error - ctrl->last_error);
    
    // 总输出
    float output = p_term + i_term + d_term;
    
    // 输出限幅
    if (output > pid_params.max_output) output = pid_params.max_output;
    if (output < -pid_params.max_output) output = -pid_params.max_output;
    
    // 死区处理
    if (abs((int)output) < pid_params.min_output && ctrl->position_error != 0) {
        output = (ctrl->position_error > 0) ? pid_params.min_output : -pid_params.min_output;
    }
    
    ctrl->last_error = ctrl->position_error;
    
    return (int16_t)output;
}

/**
 * @brief 更新控制循环
 */
void ServoControl_Update(ServoControl_t *ctrl) {
    if (!ctrl->is_enabled) return;
    
    uint32_t current_time = HAL_GetTick();
    
    // 读取当前位置
    ctrl->current_position = servo_get_position(ctrl->id);
    
    // 计算误差
    ctrl->position_error = ctrl->target_position - ctrl->current_position;
    
    // 更新状态机时间
    ServoFsm_Update(&ctrl->fsm, current_time);
    
    // 检查条件并生成事件
    ServoFsmEvent_t event = ServoFsm_CheckConditions(&ctrl->fsm, ctrl->position_error, current_time);
    
    // 处理事件
    if (event != SERVO_EVENT_NONE) {
        ServoFsmTransition_t transition = ServoFsm_ProcessEvent(&ctrl->fsm, event);
        if (transition.state_changed) {
            printf("[SERVO_CTRL] %s\r\n", transition.transition_reason);
        }
    }
    
    // 根据状态执行动作
    switch (ctrl->fsm.current_state) {
        case SERVO_FSM_IDLE:
            // 空闲状态
            break;
            
        case SERVO_FSM_MOVING:
            // PID控制
            if (abs(ctrl->position_error) > ctrl->fsm.position_tolerance) {
                int16_t speed = calculate_pid_output(ctrl);
                if (speed != 0) {
                    servo_set_speed(ctrl->id, abs(speed));
                    servo_set_position(ctrl->id, ctrl->target_position);
                }
            }
            break;
            
        case SERVO_FSM_SETTLING:
            // 稳定阶段，保持位置
            break;
            
        case SERVO_FSM_REACHED:
            // 已到达，监控位置
            if (ctrl->fsm.state_enter_time == current_time) {
                printf("[SERVO_CTRL] Position reached: %d (target: %d, error: %d)\r\n",
                       ctrl->current_position, ctrl->target_position, ctrl->position_error);
            }
            break;
            
        case SERVO_FSM_ERROR:
            // 错误状态
            ServoControl_EmergencyStop(ctrl);
            break;
    }
}

/**
 * @brief 检查是否到达目标
 */
uint8_t ServoControl_IsAtTarget(ServoControl_t *ctrl) {
    return ServoFsm_IsInState(&ctrl->fsm, SERVO_FSM_REACHED);
}

/**
 * @brief 获取移动进度
 */
float ServoControl_GetProgress(ServoControl_t *ctrl) {
    if (ServoFsm_IsInState(&ctrl->fsm, SERVO_FSM_IDLE) || 
        ServoFsm_IsInState(&ctrl->fsm, SERVO_FSM_REACHED)) {
        return 100.0f;
    }
    
    // 记录初始距离（应该在SetTargetPosition时记录）
    static int16_t initial_distance = 0;
    static int16_t last_target = 0;
    
    if (last_target != ctrl->target_position) {
        initial_distance = abs(ctrl->target_position - ctrl->current_position);
        last_target = ctrl->target_position;
    }
    
    if (initial_distance == 0) return 100.0f;
    
    int16_t moved = initial_distance - abs(ctrl->position_error);
    float progress = ((float)moved / initial_distance) * 100.0f;
    
    if (progress < 0) progress = 0;
    if (progress > 100) progress = 100;
    
    return progress;
}

/**
 * @brief 紧急停止
 */
void ServoControl_EmergencyStop(ServoControl_t *ctrl) {
    // 停在当前位置
    ctrl->target_position = ctrl->current_position;
    servo_set_position(ctrl->id, ctrl->current_position);
    ServoFsm_Reset(&ctrl->fsm);
    ctrl->error_sum = 0;
    printf("[SERVO_CTRL] Emergency stop at position %d\r\n", ctrl->current_position);
}

/**
 * @brief 相对移动
 */
void ServoControl_MoveRelative(ServoControl_t *ctrl, int16_t steps) {
    int16_t new_target = ctrl->current_position + steps;
    ServoControl_SetTargetPosition(ctrl, new_target);
}

/**
 * @brief 移动到指定角度
 */
void ServoControl_MoveToAngle(ServoControl_t *ctrl, float angle) {
    // 角度转换为步数 (360度 = 4096步)
    int16_t steps = (int16_t)(angle * 4096.0f / 360.0f);
    ServoControl_SetTargetPosition(ctrl, steps);
}

/**
 * @brief 连续旋转指定角度
 */
void ServoControl_RotateContinuous(ServoControl_t *ctrl, float degrees) {
    int16_t steps = (int16_t)(degrees * 4096.0f / 360.0f);
    ServoControl_MoveRelative(ctrl, steps);
}

/**
 * @brief 回零
 */
void ServoControl_Home(ServoControl_t *ctrl) {
    ServoControl_SetTargetPosition(ctrl, 0);
}

/**
 * @brief 打印状态
 */
void ServoControl_PrintStatus(ServoControl_t *ctrl) {
    printf("[SERVO_STATUS] ID=%d, State=%s, Pos=%d, Target=%d, Error=%d, Progress=%d%%\r\n",
           ctrl->id,
           ServoFsm_GetStateName(ctrl->fsm.current_state),
           ctrl->current_position,
           ctrl->target_position,
           ctrl->position_error,
           (int)ServoControl_GetProgress(ctrl));
}