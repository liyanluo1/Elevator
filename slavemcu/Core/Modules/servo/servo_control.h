#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "servo.h"
#include "servo_fsm.h"
#include <stdint.h>

// 控制参数
#define SERVO_CONTROL_PERIOD_MS    50      // 控制周期 50ms

// 控制结构体
typedef struct {
    uint8_t id;                 // 舵机ID
    int16_t target_position;    // 目标位置
    int16_t current_position;   // 当前位置
    int16_t position_error;     // 位置误差
    int16_t last_error;         // 上次误差
    int16_t error_sum;          // 误差累积（用于积分）
    uint16_t speed;             // 运行速度
    ServoFsm_t fsm;             // 状态机
    uint32_t move_start_time;   // 开始移动时间
    uint8_t is_enabled;         // 是否使能
} ServoControl_t;

// PID参数
typedef struct {
    float kp;   // 比例系数
    float ki;   // 积分系数
    float kd;   // 微分系数
    int16_t max_output;  // 最大输出限制
    int16_t min_output;  // 最小输出限制
} PIDParams_t;

// 函数声明
void ServoControl_Init(ServoControl_t *ctrl, uint8_t id);
void ServoControl_SetPID(PIDParams_t *pid);
void ServoControl_Enable(ServoControl_t *ctrl);
void ServoControl_Disable(ServoControl_t *ctrl);
void ServoControl_SetTargetPosition(ServoControl_t *ctrl, int16_t position);
void ServoControl_SetSpeed(ServoControl_t *ctrl, uint16_t speed);
void ServoControl_Update(ServoControl_t *ctrl);
uint8_t ServoControl_IsAtTarget(ServoControl_t *ctrl);
float ServoControl_GetProgress(ServoControl_t *ctrl);
void ServoControl_EmergencyStop(ServoControl_t *ctrl);

// 高级控制函数
void ServoControl_MoveRelative(ServoControl_t *ctrl, int16_t steps);
void ServoControl_MoveToAngle(ServoControl_t *ctrl, float angle);
void ServoControl_RotateContinuous(ServoControl_t *ctrl, float degrees);
void ServoControl_Home(ServoControl_t *ctrl);

// 调试函数
void ServoControl_PrintStatus(ServoControl_t *ctrl);

#endif /* SERVO_CONTROL_H */