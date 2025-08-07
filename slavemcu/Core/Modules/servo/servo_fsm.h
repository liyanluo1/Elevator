#ifndef SERVO_FSM_H
#define SERVO_FSM_H

#include <stdint.h>

// 状态定义
typedef enum {
    SERVO_FSM_IDLE = 0,        // 空闲
    SERVO_FSM_MOVING,          // 移动中
    SERVO_FSM_SETTLING,        // 稳定中
    SERVO_FSM_REACHED,         // 到达目标
    SERVO_FSM_ERROR            // 错误
} ServoFsmState_t;

// 事件定义
typedef enum {
    SERVO_EVENT_NONE = 0,      // 无事件
    SERVO_EVENT_START_MOVE,    // 开始移动
    SERVO_EVENT_NEAR_TARGET,   // 接近目标
    SERVO_EVENT_AT_TARGET,     // 到达目标
    SERVO_EVENT_POSITION_DRIFT, // 位置偏离
    SERVO_EVENT_TIMEOUT,       // 超时
    SERVO_EVENT_ERROR          // 错误
} ServoFsmEvent_t;

// FSM上下文
typedef struct {
    ServoFsmState_t current_state;    // 当前状态
    ServoFsmState_t previous_state;   // 前一个状态
    uint32_t state_enter_time;        // 进入状态时间
    uint32_t state_duration;          // 在当前状态持续时间
    uint8_t retry_count;              // 重试计数
    uint8_t max_retries;              // 最大重试次数
    
    // 配置参数
    uint16_t position_tolerance;      // 位置容差
    uint32_t settle_time_ms;          // 稳定时间
    uint32_t timeout_ms;              // 超时时间
} ServoFsm_t;

// 状态转换结果
typedef struct {
    uint8_t state_changed;            // 状态是否改变
    ServoFsmState_t new_state;        // 新状态
    const char* transition_reason;    // 转换原因
} ServoFsmTransition_t;

// 函数声明
void ServoFsm_Init(ServoFsm_t* fsm);
void ServoFsm_Reset(ServoFsm_t* fsm);
ServoFsmTransition_t ServoFsm_ProcessEvent(ServoFsm_t* fsm, ServoFsmEvent_t event);
void ServoFsm_Update(ServoFsm_t* fsm, uint32_t current_time);
ServoFsmEvent_t ServoFsm_CheckConditions(ServoFsm_t* fsm, int16_t position_error, uint32_t current_time);
const char* ServoFsm_GetStateName(ServoFsmState_t state);
const char* ServoFsm_GetEventName(ServoFsmEvent_t event);
uint8_t ServoFsm_IsInState(ServoFsm_t* fsm, ServoFsmState_t state);
uint8_t ServoFsm_CanMove(ServoFsm_t* fsm);

#endif /* SERVO_FSM_H */