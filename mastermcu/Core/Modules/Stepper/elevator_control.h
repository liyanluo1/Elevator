#ifndef __ELEVATOR_CONTROL_H
#define __ELEVATOR_CONTROL_H

#include "stepper_motor.h"
#include <stdint.h>
#include <stdbool.h>

/* 电梯常量定义 */
#define ELEVATOR_STEPS_PER_FLOOR   17000   // 每层楼的步数
#define ELEVATOR_MIN_FLOOR          1       // 最低楼层
#define ELEVATOR_MAX_FLOOR          10      // 最高楼层

/* 电梯状态枚举 */
typedef enum {
    ELEVATOR_STATE_IDLE = 0,        // 空闲
    ELEVATOR_STATE_MOVING           // 运行中
} ElevatorState_t;

/* 电梯控制结构体 */
typedef struct {
    /* 底层电机 */
    StepperMotor_t motor;
    
    /* 楼层信息 */
    int8_t current_floor;           // 当前楼层
    int8_t target_floor;            // 目标楼层
    
    /* 状态管理 */
    ElevatorState_t state;          // 当前状态
    ElevatorState_t prev_state;     // 上一个状态
    uint32_t state_enter_time;      // 进入状态时间
    
    /* 位置校正 */
    int32_t floor_positions[ELEVATOR_MAX_FLOOR + 1];  // 各楼层位置
    bool is_calibrated;             // 是否已校准
    
} ElevatorControl_t;

/* 全局电梯实例 */
extern ElevatorControl_t g_elevator;

/* 初始化和控制 */
void ElevatorControl_Init(void);
void ElevatorControl_Reset(void);

/* 楼层移动 */
void ElevatorControl_MoveToFloor(int8_t floor);
void ElevatorControl_MoveFloors(int8_t floors);
void ElevatorControl_Stop(void);

/* 位置校准 */
void ElevatorControl_Calibrate(int8_t floor);
void ElevatorControl_SetCurrentFloor(int8_t floor);

/* 状态查询 */
ElevatorState_t ElevatorControl_GetState(void);
int8_t ElevatorControl_GetCurrentFloor(void);
int8_t ElevatorControl_GetTargetFloor(void);
bool ElevatorControl_IsMoving(void);
bool ElevatorControl_IsIdle(void);

/* 主循环更新 - 必须定期调用 */
void ElevatorControl_Update(void);

/* 状态名称 */
const char* ElevatorControl_GetStateName(ElevatorState_t state);

#endif /* __ELEVATOR_CONTROL_H */