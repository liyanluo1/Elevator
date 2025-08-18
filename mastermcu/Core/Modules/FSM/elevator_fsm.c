#include "elevator_fsm.h"
#include "../RS485/rs485.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* 私有变量 */
static uint32_t door_operation_start = 0;
static bool door_command_sent = false;

/* ==================== 初始化 ==================== */

void FSM_Init(void) {
    Blackboard_Init();
    printf("[FSM] Initialized\r\n");
}

/* ==================== 主处理函数 ==================== */

void FSM_Process(void) {
    /* 根据当前状态执行相应处理 */
    switch (g_blackboard.state) {
        case STATE_IDLE:
            FSM_StateIdle();
            break;
            
        case STATE_MOVING:
            FSM_StateMoving();
            break;
            
        case STATE_DOOR_OPERATING:
            // 门控禁用 - 直接回IDLE
            printf("[FSM] DOOR_OPERATING disabled, back to IDLE\r\n");
            Blackboard_SetState(STATE_IDLE);
            break;
            
        case STATE_PREPARING:
            FSM_StatePreparing();
            break;
            
        default:
            Blackboard_SetState(STATE_IDLE);
            break;
    }
}

/* ==================== 事件处理 ==================== */

void FSM_HandleButtonUp(uint8_t floor) {
    printf("[FSM] Button UP pressed at floor %d\r\n", floor);
    Blackboard_AddUpCall(floor);
    
    /* 如果电梯空闲，立即响应 */
    if (g_blackboard.state == STATE_IDLE) {
        Blackboard_PushEvent(EVENT_BUTTON_UP, floor);
    }
}

void FSM_HandleButtonDown(uint8_t floor) {
    printf("[FSM] Button DOWN pressed at floor %d\r\n", floor);
    Blackboard_AddDownCall(floor);
    
    /* 如果电梯空闲，立即响应 */
    if (g_blackboard.state == STATE_IDLE) {
        Blackboard_PushEvent(EVENT_BUTTON_DOWN, floor);
    }
}

void FSM_HandleCabinCall(uint8_t floor) {
    printf("[FSM] Cabin call for floor %d\r\n", floor);
    Blackboard_AddCabinCall(floor);
    
    /* 如果电梯空闲，立即响应 */
    if (g_blackboard.state == STATE_IDLE) {
        Blackboard_PushEvent(EVENT_CABIN_CALL, floor);
    }
}

void FSM_HandlePhotoSensor(uint8_t floor) {
    printf("[FSM] Photo sensor triggered at floor %d\r\n", floor);
    g_blackboard.last_photo_floor = floor;
    
    /* 如果到达目标楼层 - 立即停止并开门 */
    if (g_blackboard.state == STATE_MOVING && floor == g_blackboard.target_floor) {
        printf("[FSM] Target floor reached! Stopping motor and opening door.\r\n");
        
        /* 立即停止电机 */
        Blackboard_SetMotorCommand(MOTOR_CMD_STOP, 0);
        
        /* 校准位置到精确楼层 */
        Blackboard_CalibratePosition(floor);
        g_blackboard.current_floor = floor;
        
        /* 清除当前楼层的请求 */
        Blackboard_ClearCall(floor);
        
        /* 发送停止方向命令 */
        FSM_SendDirectionCommand(DIR_IDLE);
        
        /* 门控禁用 - 直接回到IDLE状态 */
        printf("[FSM] Door disabled - returning to IDLE\r\n");
        Blackboard_SetState(STATE_IDLE);
        sprintf(g_blackboard.debug_msg, "Stopped at F%d", floor);
    } 
    /* 如果只是经过中间楼层 - 仅校准 */
    else if (g_blackboard.state == STATE_MOVING) {
        printf("[FSM] Passing floor %d, calibrating position\r\n", floor);
        
        /* 校准位置但不停止 */
        Blackboard_CalibratePosition(floor);
        g_blackboard.current_floor = floor;
    }
}

/* ==================== 状态处理函数 ==================== */

void FSM_StateIdle(void) {
    /* 检查是否有请求 */
    if (Blackboard_HasAnyCall()) {
        uint8_t next_floor = Blackboard_GetNextTarget();
        
        if (next_floor > 0 && next_floor != g_blackboard.current_floor) {
            /* 设置目标楼层 */
            g_blackboard.target_floor = next_floor;
            
            /* 确定方向 */
            if (next_floor > g_blackboard.current_floor) {
                g_blackboard.direction = DIR_UP;
            } else {
                g_blackboard.direction = DIR_DOWN;
            }
            
            /* 发送方向到从机 */
            FSM_SendDirectionCommand(g_blackboard.direction);
            
            /* 发送电机移动命令 */
            Blackboard_SetMotorCommand(MOTOR_CMD_MOVE_TO_FLOOR, next_floor);
            
            /* 转换到移动状态 */
            Blackboard_SetState(STATE_MOVING);
            sprintf(g_blackboard.debug_msg, "Go F%d %s", next_floor, 
                    g_blackboard.direction == DIR_UP ? "UP" : "DOWN");
                    
        } else if (next_floor == g_blackboard.current_floor) {
            /* 就在当前楼层，清除呼叫即可 */
            Blackboard_ClearCall(g_blackboard.current_floor);
            printf("[FSM] Already at floor %d, call cleared\r\n", g_blackboard.current_floor);
            sprintf(g_blackboard.debug_msg, "Already at F%d", g_blackboard.current_floor);
        }
    }
}

void FSM_StateMoving(void) {
    /* 电机位置应由主程序更新，不在FSM中直接访问 */
    
    /* 备用停止逻辑：仅在接近目标但没有光电传感器触发时使用 */
    /* 正常情况下，光电传感器会先触发并处理停止 */
    if (Blackboard_IsNearTarget()) {
        /* 检查是否已经在转换状态（避免重复） */
        if (g_blackboard.state == STATE_MOVING) {
            printf("[FSM] Backup stop: encoder position reached\r\n");
            
            /* 停止电机 */
            Blackboard_SetMotorCommand(MOTOR_CMD_STOP, 0);
            
            /* 更新当前楼层 */
            g_blackboard.current_floor = g_blackboard.target_floor;
            
            /* 清除当前楼层的请求 */
            Blackboard_ClearCall(g_blackboard.current_floor);
            
            /* 发送停止方向命令 */
            FSM_SendDirectionCommand(DIR_IDLE);
            
            /* 门控禁用 - 直接回到IDLE状态 */
            Blackboard_SetState(STATE_IDLE);
            printf("[FSM] Encoder stop, door disabled - back to IDLE\r\n");
            sprintf(g_blackboard.debug_msg, "Encoder stop F%d", g_blackboard.current_floor);
        }
    }
    
    /* 检查中途是否有同方向请求（SCAN算法） */
    if (g_blackboard.direction == DIR_UP) {
        /* 检查是否经过有请求的楼层 */
        int32_t current_pos = g_blackboard.motor_position;
        for (uint8_t floor = g_blackboard.current_floor + 1; floor < g_blackboard.target_floor; floor++) {
            if (Blackboard_HasCallAt(floor)) {
                int32_t floor_pos = Blackboard_GetTargetPosition(floor);
                if (abs(current_pos - floor_pos) < 500) {  // 接近该楼层
                    /* 更新目标为中途楼层 */
                    g_blackboard.target_floor = floor;
                    Blackboard_SetMotorCommand(MOTOR_CMD_MOVE_TO_FLOOR, floor);
                    sprintf(g_blackboard.debug_msg, "Stop at F%d", floor);
                    break;
                }
            }
        }
    } else if (g_blackboard.direction == DIR_DOWN) {
        /* 下行时类似处理 */
        int32_t current_pos = g_blackboard.motor_position;
        for (uint8_t floor = g_blackboard.current_floor - 1; floor > g_blackboard.target_floor; floor--) {
            if (Blackboard_HasCallAt(floor)) {
                int32_t floor_pos = Blackboard_GetTargetPosition(floor);
                if (abs(current_pos - floor_pos) < 500) {
                    g_blackboard.target_floor = floor;
                    Blackboard_SetMotorCommand(MOTOR_CMD_MOVE_TO_FLOOR, floor);
                    sprintf(g_blackboard.debug_msg, "Stop at F%d", floor);
                    break;
                }
            }
        }
    }
}


void FSM_StateDoorOperating(void) {
    uint32_t elapsed = HAL_GetTick() - door_operation_start;
    
    /* 阶段1：发送开门命令 */
    if (!door_command_sent) {
        FSM_SendDoorCommand(true);  // 开门
        door_command_sent = true;
        g_blackboard.door_state = DOOR_OPENING;
        sprintf(g_blackboard.debug_msg, "Door opening");
    }
    /* 阶段2：等待开门完成 */
    else if (elapsed < DOOR_OPENING_TIME_MS) {
        // 等待开门
        g_blackboard.door_state = DOOR_OPENING;
    }
    /* 阶段3：保持开门 */
    else if (elapsed < (DOOR_OPENING_TIME_MS + DOOR_OPEN_HOLD_TIME_MS)) {
        g_blackboard.door_state = DOOR_OPEN;
        sprintf(g_blackboard.debug_msg, "Door open");
    }
    /* 阶段4：发送关门命令 */
    else if (elapsed < (DOOR_OPENING_TIME_MS + DOOR_OPEN_HOLD_TIME_MS + 100)) {
        if (g_blackboard.door_state != DOOR_CLOSING) {
            FSM_SendDoorCommand(false);  // 关门
            g_blackboard.door_state = DOOR_CLOSING;
            sprintf(g_blackboard.debug_msg, "Door closing");
        }
    }
    /* 阶段5：等待关门完成 */
    else if (elapsed < (DOOR_OPENING_TIME_MS + DOOR_OPEN_HOLD_TIME_MS + DOOR_CLOSING_TIME_MS)) {
        // 等待关门
        g_blackboard.door_state = DOOR_CLOSING;
    }
    /* 阶段6：门操作完成 */
    else {
        g_blackboard.door_state = DOOR_CLOSED;
        Blackboard_SetState(STATE_PREPARING);
        sprintf(g_blackboard.debug_msg, "Door closed");
    }
}

void FSM_StatePreparing(void) {
    /* 更新方向 */
    Blackboard_UpdateDirection();
    
    /* 检查是否还有请求 */
    if (Blackboard_HasAnyCall()) {
        /* 获取下一个目标 */
        uint8_t next_floor = Blackboard_GetNextTarget();
        
        if (next_floor > 0 && next_floor != g_blackboard.current_floor) {
            /* 设置新目标 */
            g_blackboard.target_floor = next_floor;
            
            /* 确定方向 */
            if (next_floor > g_blackboard.current_floor) {
                g_blackboard.direction = DIR_UP;
            } else {
                g_blackboard.direction = DIR_DOWN;
            }
            
            /* 发送方向 */
            FSM_SendDirectionCommand(g_blackboard.direction);
            
            /* 发送电机移动命令 */
            Blackboard_SetMotorCommand(MOTOR_CMD_MOVE_TO_FLOOR, next_floor);
            
            /* 转换到移动状态 */
            Blackboard_SetState(STATE_MOVING);
            sprintf(g_blackboard.debug_msg, "Next F%d", next_floor);
        } else {
            /* 没有其他请求或就在当前楼层 */
            Blackboard_SetState(STATE_IDLE);
            g_blackboard.direction = DIR_IDLE;
            sprintf(g_blackboard.debug_msg, "Idle");
        }
    } else {
        /* 没有请求，返回空闲 */
        Blackboard_SetState(STATE_IDLE);
        g_blackboard.direction = DIR_IDLE;
        sprintf(g_blackboard.debug_msg, "Idle");
    }
}

/* ==================== 辅助函数 ==================== */

bool FSM_IsTimeout(uint32_t start_time, uint32_t timeout_ms) {
    return (HAL_GetTick() - start_time) >= timeout_ms;
}

void FSM_SendDoorCommand(bool open) {
    uint8_t tx_buffer[4];
    tx_buffer[0] = open ? CMD_DOOR_OPEN : CMD_DOOR_CLOSE;
    tx_buffer[1] = 0;
    tx_buffer[2] = 0;
    tx_buffer[3] = 0;
    
    rs485_send_packet_dma(tx_buffer, 4);
    printf("[FSM] Door command sent: %s\r\n", open ? "OPEN" : "CLOSE");
}

void FSM_SendDirectionCommand(Direction_t dir) {
    uint8_t tx_buffer[4];
    tx_buffer[0] = CMD_DIRECTION_SET;
    tx_buffer[1] = dir;  // 0=IDLE, 1=UP, 2=DOWN
    tx_buffer[2] = 0;
    tx_buffer[3] = 0;
    
    rs485_send_packet_dma(tx_buffer, 4);
    printf("[FSM] Direction set: %d\r\n", dir);
}