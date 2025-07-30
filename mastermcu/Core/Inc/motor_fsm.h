#ifndef __MOTOR_FSM_H
#define __MOTOR_FSM_H

#include "motor_advanced.h"
#include "blackboard.h"
#include <stdbool.h>

// 电机状态枚举
typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_ACCELERATING,
    MOTOR_STATE_CRUISING,
    MOTOR_STATE_WAIT_CALIBRATE,
    MOTOR_STATE_ADJUST,
    MOTOR_STATE_TIMEOUT,
    MOTOR_STATE_ERROR
} MotorState_t;

// 电机FSM控制结构体
typedef struct {
    MotorState_t current_state;
    MotorState_t prev_state;
    
    MotorAdvanced_t* motor_adv;
    
    // 私有变量
    uint32_t start_time;
    int pulse_count;            // 当前脉冲计数
    int expected_pulses;        // 预期脉冲数
    int retry_count;            // 重试计数
    
    // 速度控制
    int current_speed_percent;  // 当前速度百分比
    int target_speed_percent;   // 目标速度百分比
    
    // 位置控制
    int32_t start_position;     // 起始位置
    int32_t target_position;    // 目标位置
    int32_t position_error;     // 位置误差
    
    // 校准相关
    bool wait_for_calibration;
    uint32_t calibration_timeout;
    
} MotorFSM_t;

// 全局电机FSM实例
extern MotorFSM_t g_motor_fsm;

// 初始化
void MotorFSM_Init(MotorFSM_t* fsm, MotorAdvanced_t* motor_adv);

// 主更新函数
void MotorFSM_Update(MotorFSM_t* fsm);

// 启动函数
void MotorFSM_StartMoving(MotorFSM_t* fsm, Direction_t dir, int speed_percent);
void MotorFSM_Stop(MotorFSM_t* fsm);
void MotorFSM_AdjustPosition(MotorFSM_t* fsm, int offset);

// 内部函数
void MotorFSM_GeneratePulse(MotorFSM_t* fsm, Direction_t dir, int speed);
int MotorFSM_CalculateAdjustSteps(MotorFSM_t* fsm, int offset, Direction_t dir);
void MotorFSM_SendCANCommand(MotorFSM_t* fsm, const char* cmd);

// 状态查询
bool MotorFSM_IsIdle(MotorFSM_t* fsm);
bool MotorFSM_HasError(MotorFSM_t* fsm);
int MotorFSM_GetPosition(MotorFSM_t* fsm);

#endif /* __MOTOR_FSM_H */