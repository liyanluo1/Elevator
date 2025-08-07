#include "servo_fsm.h"
#include <stdio.h>
#include "stm32f1xx_hal.h"

// 默认配置
#define DEFAULT_POSITION_TOLERANCE  5      // 默认容差5步
#define DEFAULT_SETTLE_TIME_MS      500    // 默认稳定时间500ms
#define DEFAULT_TIMEOUT_MS          10000  // 默认超时10秒
#define DEFAULT_MAX_RETRIES         3      // 默认最大重试3次

/**
 * @brief 初始化状态机
 */
void ServoFsm_Init(ServoFsm_t* fsm) {
    fsm->current_state = SERVO_FSM_IDLE;
    fsm->previous_state = SERVO_FSM_IDLE;
    fsm->state_enter_time = 0;
    fsm->state_duration = 0;
    fsm->retry_count = 0;
    fsm->max_retries = DEFAULT_MAX_RETRIES;
    
    // 默认参数
    fsm->position_tolerance = DEFAULT_POSITION_TOLERANCE;
    fsm->settle_time_ms = DEFAULT_SETTLE_TIME_MS;
    fsm->timeout_ms = DEFAULT_TIMEOUT_MS;
}

/**
 * @brief 重置状态机
 */
void ServoFsm_Reset(ServoFsm_t* fsm) {
    fsm->current_state = SERVO_FSM_IDLE;
    fsm->previous_state = SERVO_FSM_IDLE;
    fsm->state_enter_time = HAL_GetTick();
    fsm->state_duration = 0;
    fsm->retry_count = 0;
}

/**
 * @brief 获取状态名称
 */
const char* ServoFsm_GetStateName(ServoFsmState_t state) {
    static const char* state_names[] = {
        "IDLE",
        "MOVING", 
        "SETTLING",
        "REACHED",
        "ERROR"
    };
    
    if (state < sizeof(state_names)/sizeof(state_names[0])) {
        return state_names[state];
    }
    return "UNKNOWN";
}

/**
 * @brief 获取事件名称
 */
const char* ServoFsm_GetEventName(ServoFsmEvent_t event) {
    static const char* event_names[] = {
        "NONE",
        "START_MOVE",
        "NEAR_TARGET",
        "AT_TARGET",
        "POSITION_DRIFT",
        "TIMEOUT",
        "ERROR"
    };
    
    if (event < sizeof(event_names)/sizeof(event_names[0])) {
        return event_names[event];
    }
    return "UNKNOWN";
}

/**
 * @brief 状态转换
 */
static void change_state(ServoFsm_t* fsm, ServoFsmState_t new_state, uint32_t current_time) {
    if (fsm->current_state != new_state) {
        fsm->previous_state = fsm->current_state;
        fsm->current_state = new_state;
        fsm->state_enter_time = current_time;
        fsm->state_duration = 0;
        
        printf("[FSM] State: %s -> %s\r\n", 
               ServoFsm_GetStateName(fsm->previous_state),
               ServoFsm_GetStateName(fsm->current_state));
    }
}

/**
 * @brief 处理事件
 */
ServoFsmTransition_t ServoFsm_ProcessEvent(ServoFsm_t* fsm, ServoFsmEvent_t event) {
    ServoFsmTransition_t result = {0};
    ServoFsmState_t old_state = fsm->current_state;
    uint32_t current_time = HAL_GetTick();
    
    switch (fsm->current_state) {
        case SERVO_FSM_IDLE:
            if (event == SERVO_EVENT_START_MOVE) {
                change_state(fsm, SERVO_FSM_MOVING, current_time);
                fsm->retry_count = 0;
                result.transition_reason = "Start movement";
            }
            break;
            
        case SERVO_FSM_MOVING:
            if (event == SERVO_EVENT_NEAR_TARGET) {
                change_state(fsm, SERVO_FSM_SETTLING, current_time);
                result.transition_reason = "Near target, settling";
            } else if (event == SERVO_EVENT_TIMEOUT) {
                change_state(fsm, SERVO_FSM_ERROR, current_time);
                result.transition_reason = "Movement timeout";
            } else if (event == SERVO_EVENT_AT_TARGET) {
                // 直接到达（误差很小）
                change_state(fsm, SERVO_FSM_SETTLING, current_time);
                result.transition_reason = "At target";
            }
            break;
            
        case SERVO_FSM_SETTLING:
            if (event == SERVO_EVENT_AT_TARGET) {
                if (fsm->state_duration >= fsm->settle_time_ms) {
                    change_state(fsm, SERVO_FSM_REACHED, current_time);
                    result.transition_reason = "Position stable";
                }
            } else if (event == SERVO_EVENT_POSITION_DRIFT) {
                fsm->retry_count++;
                if (fsm->retry_count > fsm->max_retries) {
                    change_state(fsm, SERVO_FSM_ERROR, current_time);
                    result.transition_reason = "Max retries exceeded";
                } else {
                    change_state(fsm, SERVO_FSM_MOVING, current_time);
                    result.transition_reason = "Position drift, retry";
                }
            }
            break;
            
        case SERVO_FSM_REACHED:
            if (event == SERVO_EVENT_POSITION_DRIFT) {
                // 位置偏离太大，重新调整
                change_state(fsm, SERVO_FSM_MOVING, current_time);
                result.transition_reason = "Position drift detected";
            } else if (event == SERVO_EVENT_START_MOVE) {
                // 新的移动命令
                change_state(fsm, SERVO_FSM_MOVING, current_time);
                fsm->retry_count = 0;
                result.transition_reason = "New movement";
            }
            break;
            
        case SERVO_FSM_ERROR:
            if (event == SERVO_EVENT_START_MOVE) {
                // 从错误状态恢复
                change_state(fsm, SERVO_FSM_IDLE, current_time);
                fsm->retry_count = 0;
                result.transition_reason = "Reset from error";
            }
            break;
    }
    
    result.state_changed = (old_state != fsm->current_state);
    result.new_state = fsm->current_state;
    
    return result;
}

/**
 * @brief 根据条件检查应该触发的事件
 */
ServoFsmEvent_t ServoFsm_CheckConditions(ServoFsm_t* fsm, int16_t position_error, uint32_t current_time) {
    uint16_t abs_error = (position_error < 0) ? -position_error : position_error;
    
    switch (fsm->current_state) {
        case SERVO_FSM_MOVING:
            if (abs_error <= fsm->position_tolerance) {
                return SERVO_EVENT_NEAR_TARGET;
            }
            if (fsm->state_duration > fsm->timeout_ms) {
                return SERVO_EVENT_TIMEOUT;
            }
            break;
            
        case SERVO_FSM_SETTLING:
            if (abs_error > fsm->position_tolerance) {
                return SERVO_EVENT_POSITION_DRIFT;
            }
            if (abs_error <= fsm->position_tolerance && 
                fsm->state_duration >= fsm->settle_time_ms) {
                return SERVO_EVENT_AT_TARGET;
            }
            break;
            
        case SERVO_FSM_REACHED:
            // 检查是否偏离过多（容差的2倍）
            if (abs_error > fsm->position_tolerance * 2) {
                return SERVO_EVENT_POSITION_DRIFT;
            }
            break;
            
        default:
            break;
    }
    
    return SERVO_EVENT_NONE;
}

/**
 * @brief 更新状态机时间
 */
void ServoFsm_Update(ServoFsm_t* fsm, uint32_t current_time) {
    fsm->state_duration = current_time - fsm->state_enter_time;
}

/**
 * @brief 检查是否在指定状态
 */
uint8_t ServoFsm_IsInState(ServoFsm_t* fsm, ServoFsmState_t state) {
    return (fsm->current_state == state);
}

/**
 * @brief 检查是否可以移动
 */
uint8_t ServoFsm_CanMove(ServoFsm_t* fsm) {
    return (fsm->current_state != SERVO_FSM_ERROR);
}