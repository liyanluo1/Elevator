#ifndef __MOTOR_ADVANCED_H
#define __MOTOR_ADVANCED_H

#include "motor_control.h"
#include <stdbool.h>
#include <stdint.h>

// 速度曲线阶段
typedef enum {
    SPEED_STAGE_IDLE = 0,
    SPEED_STAGE_ACCEL,      // 加速阶段
    SPEED_STAGE_CRUISE,     // 匀速阶段
    SPEED_STAGE_DECEL       // 减速阶段
} SpeedStage_t;

// 速度曲线参数
typedef struct {
    uint16_t start_speed;   // 起始速度
    uint16_t cruise_speed;  // 巡航速度
    uint16_t end_speed;     // 结束速度
    uint16_t accel_rate;    // 加速度
    uint16_t decel_rate;    // 减速度
    uint32_t accel_steps;   // 加速步数
    uint32_t decel_steps;   // 减速步数
    uint32_t cruise_steps;  // 匀速步数
} SpeedProfile_t;

// 高级电机控制结构体
typedef struct {
    Motor_t* motor;             // 基础电机结构体
    SpeedProfile_t profile;     // 速度曲线
    SpeedStage_t stage;         // 当前速度阶段
    
    // 位置控制
    int32_t cmd_position;       // 命令位置
    int32_t actual_position;    // 实际位置（脉冲计数）
    int32_t position_error;     // 位置误差
    
    // 速度控制
    uint16_t current_speed;     // 当前速度
    uint32_t stage_steps;       // 当前阶段已走步数
    uint32_t total_steps;       // 总步数
    
    // 时间管理
    uint32_t last_update_time;  // 上次更新时间
    uint32_t move_start_time;   // 运动开始时间
    
    // 校准相关
    bool is_calibrated;         // 是否已校准
    bool is_homing;             // 是否在归位中
    int32_t home_offset;        // 零点偏移
    
} MotorAdvanced_t;

// 初始化函数
void MotorAdv_Init(MotorAdvanced_t* motor_adv, Motor_t* motor);

// 速度曲线计算
void MotorAdv_CalculateProfile(MotorAdvanced_t* motor_adv, int32_t target_position);
void MotorAdv_UpdateSpeed(MotorAdvanced_t* motor_adv);

// 位置控制
void MotorAdv_MoveToPosition(MotorAdvanced_t* motor_adv, int32_t position);
void MotorAdv_MoveRelative(MotorAdvanced_t* motor_adv, int32_t steps);
void MotorAdv_Stop(MotorAdvanced_t* motor_adv);
void MotorAdv_EmergencyStop(MotorAdvanced_t* motor_adv);

// 归位和校准
void MotorAdv_StartHoming(MotorAdvanced_t* motor_adv);
void MotorAdv_SetHomePosition(MotorAdvanced_t* motor_adv);
bool MotorAdv_IsHoming(MotorAdvanced_t* motor_adv);
bool MotorAdv_IsCalibrated(MotorAdvanced_t* motor_adv);

// 位置更新和误差校正
void MotorAdv_UpdatePosition(MotorAdvanced_t* motor_adv, int32_t sensor_position);
void MotorAdv_CorrectPosition(MotorAdvanced_t* motor_adv, int32_t actual_position);

// 状态查询
bool MotorAdv_IsAtTarget(MotorAdvanced_t* motor_adv);
int32_t MotorAdv_GetPosition(MotorAdvanced_t* motor_adv);
uint16_t MotorAdv_GetSpeed(MotorAdvanced_t* motor_adv);
SpeedStage_t MotorAdv_GetStage(MotorAdvanced_t* motor_adv);

// 主循环处理
void MotorAdv_Process(MotorAdvanced_t* motor_adv);

#endif /* __MOTOR_ADVANCED_H */