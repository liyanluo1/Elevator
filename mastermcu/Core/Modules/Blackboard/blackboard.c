#include "blackboard.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"

/* 全局Blackboard实例 */
Blackboard_t g_blackboard;

/* ==================== 初始化函数 ==================== */

void Blackboard_Init(void) {
    memset(&g_blackboard, 0, sizeof(Blackboard_t));
    
    /* 初始状态 */
    g_blackboard.state = STATE_IDLE;
    g_blackboard.current_floor = 1;  // 假设从1楼开始
    g_blackboard.target_floor = 1;
    g_blackboard.next_target_floor = 0;  // 无下一个目标
    g_blackboard.direction = DIR_IDLE;
    g_blackboard.door_state = DOOR_CLOSED;
    
    /* 清空所有呼叫 */
    for (int i = 0; i <= MAX_FLOORS; i++) {
        g_blackboard.up_calls[i] = false;
        g_blackboard.down_calls[i] = false;
        g_blackboard.cabin_calls[i] = false;
    }
    
    /* 初始化事件队列 */
    g_blackboard.event_head = 0;
    g_blackboard.event_tail = 0;
    g_blackboard.event_count = 0;
    
    strcpy(g_blackboard.debug_msg, "System Init");
    
    printf("[Blackboard] Initialized\r\n");
}

void Blackboard_Reset(void) {
    Blackboard_Init();
}

/* ==================== 状态管理 ==================== */

void Blackboard_SetState(ElevatorState_t new_state) {
    if (g_blackboard.state != new_state) {
        g_blackboard.prev_state = g_blackboard.state;
        g_blackboard.state = new_state;
        g_blackboard.state_enter_time = HAL_GetTick();
        
        printf("[Blackboard] State: %s -> %s\r\n", 
               Blackboard_GetStateName(g_blackboard.prev_state),
               Blackboard_GetStateName(new_state));
    }
}

const char* Blackboard_GetStateName(ElevatorState_t state) {
    switch (state) {
        case STATE_IDLE:           return "IDLE";
        case STATE_MOVING:         return "MOVING";
        case STATE_DOOR_OPERATING: return "DOOR_OP";
        case STATE_PREPARING:      return "PREPARING";
        default:                   return "UNKNOWN";
    }
}

/* ==================== 事件管理 ==================== */

bool Blackboard_PushEvent(EventType_t type, uint8_t floor) {
    if (g_blackboard.event_count >= 16) {
        printf("[Blackboard] Event queue full!\r\n");
        return false;
    }
    
    Event_t* event = &g_blackboard.events[g_blackboard.event_tail];
    event->type = type;
    event->floor = floor;
    event->timestamp = HAL_GetTick();
    
    g_blackboard.event_tail = (g_blackboard.event_tail + 1) % 16;
    g_blackboard.event_count++;
    
    printf("[Blackboard] Event pushed: type=%d, floor=%d\r\n", type, floor);
    return true;
}

bool Blackboard_PopEvent(Event_t* event) {
    if (g_blackboard.event_count == 0) {
        return false;
    }
    
    *event = g_blackboard.events[g_blackboard.event_head];
    g_blackboard.event_head = (g_blackboard.event_head + 1) % 16;
    g_blackboard.event_count--;
    
    return true;
}

bool Blackboard_HasEvents(void) {
    return g_blackboard.event_count > 0;
}

void Blackboard_ClearEvents(void) {
    g_blackboard.event_head = 0;
    g_blackboard.event_tail = 0;
    g_blackboard.event_count = 0;
}

/* ==================== 请求管理 ==================== */

void Blackboard_AddUpCall(uint8_t floor) {
    /* 1楼上行按钮已启用 */
    if (floor >= 1 && floor <= MAX_FLOORS - 1) {  // 3楼没有上行
        g_blackboard.up_calls[floor] = true;
        printf("[Blackboard] Up call added at floor %d\r\n", floor);
    }
}

void Blackboard_AddDownCall(uint8_t floor) {
    if (floor >= 2 && floor <= MAX_FLOORS) {  // 1楼没有下行
        g_blackboard.down_calls[floor] = true;
        printf("[Blackboard] Down call added at floor %d\r\n", floor);
    }
}

void Blackboard_AddCabinCall(uint8_t floor) {
    if (floor >= 1 && floor <= MAX_FLOORS) {
        g_blackboard.cabin_calls[floor] = true;
        printf("[Blackboard] Cabin call added for floor %d\r\n", floor);
    }
}

void Blackboard_ClearCall(uint8_t floor) {
    if (floor >= 1 && floor <= MAX_FLOORS) {
        /* 打印清除前的状态 */
        printf("[Blackboard] Clearing calls at floor %d (before: UP=%d, DOWN=%d, CABIN=%d)\r\n", 
               floor,
               g_blackboard.up_calls[floor],
               g_blackboard.down_calls[floor],
               g_blackboard.cabin_calls[floor]);
        
        g_blackboard.up_calls[floor] = false;
        g_blackboard.down_calls[floor] = false;
        g_blackboard.cabin_calls[floor] = false;
        
        /* 打印清除后的状态 */
        printf("[Blackboard] Calls cleared at floor %d (after: UP=%d, DOWN=%d, CABIN=%d)\r\n", 
               floor,
               g_blackboard.up_calls[floor],
               g_blackboard.down_calls[floor],
               g_blackboard.cabin_calls[floor]);
    } else {
        printf("[Blackboard] WARNING: Invalid floor %d for ClearCall\r\n", floor);
    }
}

bool Blackboard_HasCallAt(uint8_t floor) {
    if (floor < 1 || floor > MAX_FLOORS) return false;
    
    return g_blackboard.up_calls[floor] || 
           g_blackboard.down_calls[floor] || 
           g_blackboard.cabin_calls[floor];
}

bool Blackboard_HasAnyCall(void) {
    for (uint8_t floor = 1; floor <= MAX_FLOORS; floor++) {
        if (Blackboard_HasCallAt(floor)) {
            return true;
        }
    }
    return false;
}

/* ==================== SCAN调度算法 ==================== */

uint8_t Blackboard_GetNextTarget(void) {
    uint8_t current = g_blackboard.current_floor;
    
    /* 如果当前方向是上行 */
    if (g_blackboard.direction == DIR_UP) {
        /* 查找当前楼层以上的请求 */
        for (uint8_t floor = current + 1; floor <= MAX_FLOORS; floor++) {
            if (Blackboard_HasCallAt(floor)) {
                return floor;
            }
        }
        /* 没有上方请求，查找下方请求 */
        for (uint8_t floor = current - 1; floor >= 1; floor--) {
            if (Blackboard_HasCallAt(floor)) {
                g_blackboard.direction = DIR_DOWN;  // 改变方向
                return floor;
            }
        }
    }
    /* 如果当前方向是下行 */
    else if (g_blackboard.direction == DIR_DOWN) {
        /* 查找当前楼层以下的请求 */
        for (uint8_t floor = current - 1; floor >= 1; floor--) {
            if (Blackboard_HasCallAt(floor)) {
                return floor;
            }
        }
        /* 没有下方请求，查找上方请求 */
        for (uint8_t floor = current + 1; floor <= MAX_FLOORS; floor++) {
            if (Blackboard_HasCallAt(floor)) {
                g_blackboard.direction = DIR_UP;  // 改变方向
                return floor;
            }
        }
    }
    /* 如果当前静止 */
    else {
        /* 先查找当前楼层 */
        if (Blackboard_HasCallAt(current)) {
            return current;
        }
        /* 找最近的请求 */
        for (uint8_t dist = 1; dist < MAX_FLOORS; dist++) {
            if (current + dist <= MAX_FLOORS && Blackboard_HasCallAt(current + dist)) {
                g_blackboard.direction = DIR_UP;
                return current + dist;
            }
            if (current - dist >= 1 && Blackboard_HasCallAt(current - dist)) {
                g_blackboard.direction = DIR_DOWN;
                return current - dist;
            }
        }
    }
    
    return 0;  // 没有请求
}

bool Blackboard_HasCallsInDirection(Direction_t dir) {
    uint8_t current = g_blackboard.current_floor;
    
    if (dir == DIR_UP) {
        for (uint8_t floor = current + 1; floor <= MAX_FLOORS; floor++) {
            if (Blackboard_HasCallAt(floor)) return true;
        }
    } else if (dir == DIR_DOWN) {
        for (uint8_t floor = current - 1; floor >= 1; floor--) {
            if (Blackboard_HasCallAt(floor)) return true;
        }
    }
    
    return false;
}

void Blackboard_UpdateDirection(void) {
    /* 如果当前方向没有请求了，尝试改变方向 */
    if (!Blackboard_HasCallsInDirection(g_blackboard.direction)) {
        if (g_blackboard.direction == DIR_UP) {
            if (Blackboard_HasCallsInDirection(DIR_DOWN)) {
                g_blackboard.direction = DIR_DOWN;
            } else {
                g_blackboard.direction = DIR_IDLE;
            }
        } else if (g_blackboard.direction == DIR_DOWN) {
            if (Blackboard_HasCallsInDirection(DIR_UP)) {
                g_blackboard.direction = DIR_UP;
            } else {
                g_blackboard.direction = DIR_IDLE;
            }
        }
    }
}

/* ==================== 位置管理 ==================== */

void Blackboard_UpdateMotorPosition(int32_t position) {
    g_blackboard.motor_position = position;
}

int32_t Blackboard_GetTargetPosition(uint8_t floor) {
    if (floor < 1 || floor > MAX_FLOORS) return 0;
    
    /* 计算目标位置：offset + (floor-1) * STEPS_PER_FLOOR */
    return g_blackboard.encoder_offset + (floor - 1) * STEPS_PER_FLOOR;
}

bool Blackboard_IsNearTarget(void) {
    int32_t target_pos = Blackboard_GetTargetPosition(g_blackboard.target_floor);
    int32_t error = abs(g_blackboard.motor_position - target_pos);
    return error < 300;  // 容差300步
}

void Blackboard_CalibratePosition(uint8_t floor) {
    if (floor < 1 || floor > MAX_FLOORS) return;
    
    /* 计算理论位置和误差 */
    int32_t expected_pos = g_blackboard.encoder_offset + (floor - 1) * STEPS_PER_FLOOR;
    int32_t error = g_blackboard.motor_position - expected_pos;
    
    /* 仅记录误差，不修改位置（开环控制） */
    if (abs(error) > 1000) {
        printf("[BB] Position error at floor %d: %ld steps\r\n", floor, error);
    }
    
    /* 更新当前楼层 */
    g_blackboard.current_floor = floor;
}

void Blackboard_SetEncoderOffset(int32_t offset) {
    g_blackboard.encoder_offset = offset;
    printf("[BB] Encoder offset set to: %ld\r\n", offset);
}

/* 电机控制接口实现 */
void Blackboard_SetMotorCommand(MotorCommand_t cmd, int32_t param) {
    g_blackboard.motor_command = cmd;
    g_blackboard.motor_command_param = param;
}

MotorCommand_t Blackboard_GetMotorCommand(void) {
    return g_blackboard.motor_command;
}

int32_t Blackboard_GetMotorCommandParam(void) {
    return g_blackboard.motor_command_param;
}

void Blackboard_ClearMotorCommand(void) {
    g_blackboard.motor_command = MOTOR_CMD_NONE;
    g_blackboard.motor_command_param = 0;
}

/* ==================== 调试函数 ==================== */

void Blackboard_UpdateDebugMsg(const char* msg) {
    strncpy(g_blackboard.debug_msg, msg, 31);
    g_blackboard.debug_msg[31] = '\0';
}

void Blackboard_PrintStatus(void) {
    printf("\r\n=== Blackboard Status ===\r\n");
    printf("State: %s\r\n", Blackboard_GetStateName(g_blackboard.state));
    printf("Floor: %d -> %d\r\n", g_blackboard.current_floor, g_blackboard.target_floor);
    printf("Direction: %s\r\n", 
           g_blackboard.direction == DIR_UP ? "UP" : 
           g_blackboard.direction == DIR_DOWN ? "DOWN" : "IDLE");
    
    printf("Calls: ");
    for (uint8_t f = 1; f <= MAX_FLOORS; f++) {
        if (g_blackboard.up_calls[f]) printf("%d^ ", f);
        if (g_blackboard.down_calls[f]) printf("%dv ", f);
        if (g_blackboard.cabin_calls[f]) printf("%d* ", f);
    }
    printf("\r\n");
    printf("Debug: %s\r\n", g_blackboard.debug_msg);
    printf("========================\r\n");
}