#include "elevator_fsm.h"
#include "../RS485/rs485.h"
#include "../RS485/rs485_protocol.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"

/* 全局变量（供main.c访问） */
uint32_t door_operation_start = 0;

/* 私有变量 */
static bool door_command_sent = false;

/* ==================== 初始化 ==================== */

void FSM_Init(void) {
    Blackboard_Init();
    printf("[FSM] Initialized\r\n");
}

/* ==================== 主处理函数 ==================== */

void FSM_Process(void) {
    Event_t event;
    
    /* 事件驱动处理 - 优先处理事件队列 */
    while (Blackboard_PopEvent(&event)) {
        printf("[FSM] Processing event: type=%d, floor=%d\r\n", event.type, event.floor);
        
        /* USART2调试 */
        extern UART_HandleTypeDef huart2;
        char evt_msg[60];
        sprintf(evt_msg, "[U2-FSM] Event T:%d F:%d\r\n", event.type, event.floor);
        HAL_UART_Transmit(&huart2, (uint8_t*)evt_msg, strlen(evt_msg), 100);
        
        switch (event.type) {
            case EVENT_BUTTON_UP:
                FSM_HandleButtonUp(event.floor);
                break;
                
            case EVENT_BUTTON_DOWN:
                FSM_HandleButtonDown(event.floor);
                break;
                
            case EVENT_CABIN_CALL:
                FSM_HandleCabinCall(event.floor);
                break;
                
            case EVENT_PHOTO_SENSOR:
                FSM_HandlePhotoSensor(event.floor);
                break;
                
            case EVENT_ARRIVED:
                printf("[FSM] Arrived at floor %d, checking for next call\r\n", event.floor);
                /* 到达楼层后，检查是否有新的呼叫 */
                FSM_CheckAndStartMovement();
                break;
                
            case EVENT_TIMEOUT:
                printf("[FSM] Timeout event\r\n");
                break;
                
            default:
                printf("[FSM] Unknown event type: %d\r\n", event.type);
                break;
        }
    }
    
    /* 状态机处理 - 根据当前状态执行相应逻辑 */
    switch (g_blackboard.state) {
        case STATE_IDLE:
            FSM_StateIdle();
            break;
            
        case STATE_MOVING:
            FSM_StateMoving();
            break;
            
        case STATE_DOOR_OPERATING:
            FSM_StateDoorOperating();
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
    /* 1楼上行按钮已启用 */
    printf("[FSM] Button UP pressed at floor %d\r\n", floor);
    Blackboard_AddUpCall(floor);
    
    /* 如果电梯空闲，触发状态机检查 */
    if (g_blackboard.state == STATE_IDLE) {
        /* 直接触发响应，不再推送事件 */
        FSM_CheckAndStartMovement();
    }
}

void FSM_HandleButtonDown(uint8_t floor) {
    printf("[FSM] Button DOWN pressed at floor %d\r\n", floor);
    Blackboard_AddDownCall(floor);
    
    /* 如果电梯空闲，触发状态机检查 */
    if (g_blackboard.state == STATE_IDLE) {
        /* 直接触发响应，不再推送事件 */
        FSM_CheckAndStartMovement();
    }
}

void FSM_HandleCabinCall(uint8_t floor) {
    printf("[FSM] Cabin call for floor %d\r\n", floor);
    Blackboard_AddCabinCall(floor);
    
    /* 如果电梯空闲，触发状态机检查 */
    if (g_blackboard.state == STATE_IDLE) {
        /* 直接触发响应，不再推送事件 */
        FSM_CheckAndStartMovement();
    }
}

void FSM_HandlePhotoSensor(uint8_t floor) {
    printf("[FSM] Photo sensor triggered at floor %d (state=%s)\r\n", 
           floor, Blackboard_GetStateName(g_blackboard.state));
    g_blackboard.last_photo_floor = floor;
    
    /* 更新当前楼层，不管什么状态 */
    g_blackboard.current_floor = floor;
    
    /* 打印当前呼叫状态用于调试 */
    printf("[FSM] Calls at floor %d: UP=%d, DOWN=%d, CABIN=%d\r\n",
           floor, 
           g_blackboard.up_calls[floor],
           g_blackboard.down_calls[floor],
           g_blackboard.cabin_calls[floor]);
    
    if (g_blackboard.state == STATE_MOVING) {
        /* 情况1：到达目标楼层 */
        if (floor == g_blackboard.target_floor) {
            printf("[FSM] Target floor reached! Stopping at floor %d\r\n", floor);
            
            /* 立即停止电机 */
            Blackboard_SetMotorCommand(MOTOR_CMD_STOP, 0);
            
            /* 清除该楼层的所有呼叫 */
            printf("[FSM] Clearing ALL calls at target floor %d\r\n", floor);
            Blackboard_ClearCall(floor);
            
            /* 发送停止方向命令 */
            FSM_SendDirectionCommand(DIR_STOP);
            
            /* 门控禁用 - 改为DOOR_OPERATING状态（非阻塞） */
            printf("[FSM] Starting door operation\r\n");
            
            /* 转到门操作状态 */
            Blackboard_SetState(STATE_DOOR_OPERATING);
            door_operation_start = HAL_GetTick();  // 记录开始时间
            door_command_sent = false;
            sprintf(g_blackboard.debug_msg, "Door F%d", floor);
        }
        /* 情况2：检查中途停靠 */
        else if (Blackboard_HasCallAt(floor)) {
            bool should_stop = false;
            
            /* 检查是否有同方向的呼叫 */
            if (g_blackboard.direction == DIR_UP) {
                /* 上行时：检查上行外呼或任何内呼 */
                if (g_blackboard.up_calls[floor] || g_blackboard.cabin_calls[floor]) {
                    should_stop = true;
                }
            }
            else if (g_blackboard.direction == DIR_DOWN) {
                /* 下行时：检查下行外呼或任何内呼 */
                if (g_blackboard.down_calls[floor] || g_blackboard.cabin_calls[floor]) {
                    should_stop = true;
                }
            }
            
            if (should_stop) {
                printf("[FSM] Intermediate stop at floor %d (target: %d)\r\n", 
                       floor, g_blackboard.target_floor);
                
                /* 停止电机 */
                Blackboard_SetMotorCommand(MOTOR_CMD_STOP, 0);
                
                /* 清除适当的呼叫 */
                if (g_blackboard.direction == DIR_UP) {
                    if (g_blackboard.up_calls[floor]) {
                        g_blackboard.up_calls[floor] = false;
                        printf("[FSM] Cleared UP call at floor %d\r\n", floor);
                    }
                } else if (g_blackboard.direction == DIR_DOWN) {
                    if (g_blackboard.down_calls[floor]) {
                        g_blackboard.down_calls[floor] = false;
                        printf("[FSM] Cleared DOWN call at floor %d\r\n", floor);
                    }
                }
                if (g_blackboard.cabin_calls[floor]) {
                    g_blackboard.cabin_calls[floor] = false;
                    printf("[FSM] Cleared CABIN call at floor %d\r\n", floor);
                }
                
                /* 转到门操作状态（非阻塞） */
                printf("[FSM] Starting intermediate door operation\r\n");
                
                /* 记住要继续去的目标 */
                g_blackboard.next_target_floor = g_blackboard.target_floor;
                
                /* 转到门操作状态 */
                Blackboard_SetState(STATE_DOOR_OPERATING);
                door_operation_start = HAL_GetTick();
                door_command_sent = false;
                sprintf(g_blackboard.debug_msg, "Door@F%d", floor);
            }
            else {
                /* 有呼叫但方向不对，只更新楼层 */
                printf("[FSM] Passing floor %d (has calls but wrong direction)\r\n", floor);
            }
        }
        /* 情况3：只是路过 */
        else {
            printf("[FSM] Passing floor %d (no calls)\r\n", floor);
            
            /* 位置误差检测 */
            int32_t expected_pos = (floor - 1) * STEPS_PER_FLOOR + g_blackboard.encoder_offset;
            int32_t actual_pos = g_blackboard.motor_position;
            int32_t error = abs(actual_pos - expected_pos);
            
            if (error > 3000) {
                printf("[WARNING] Large position error at floor %d: %ld steps\r\n", floor, error);
            } else if (error > 1000) {
                printf("[INFO] Position error at floor %d: %ld steps\r\n", floor, error);
            }
        }
    }
    else if (g_blackboard.state == STATE_IDLE) {
        /* 在IDLE状态下，如果光电触发且有呼叫，也清除它 */
        if (Blackboard_HasCallAt(floor)) {
            printf("[FSM] Clearing calls at floor %d in IDLE state\r\n", floor);
            Blackboard_ClearCall(floor);
        }
    }
}

/* ==================== 状态处理函数 ==================== */

void FSM_StateIdle(void) {
    /* IDLE状态下不主动检查，等待事件触发 */
    /* 这使得系统真正成为事件驱动 */
}

/* 新增：检查并启动移动的辅助函数 */
void FSM_CheckAndStartMovement(void) {
    /* USART2调试 */
    extern UART_HandleTypeDef huart2;
    char check_msg[80];
    sprintf(check_msg, "[U2-CHECK] State:%d HasCall:%d\r\n", 
            g_blackboard.state, Blackboard_HasAnyCall());
    HAL_UART_Transmit(&huart2, (uint8_t*)check_msg, strlen(check_msg), 100);
    
    /* 只在IDLE状态下检查是否有请求 */
    if (g_blackboard.state != STATE_IDLE) {
        return;
    }
    
    if (Blackboard_HasAnyCall()) {
        uint8_t next_floor = Blackboard_GetNextTarget();
        
        sprintf(check_msg, "[U2-CHECK] Next target: %d\r\n", next_floor);
        HAL_UART_Transmit(&huart2, (uint8_t*)check_msg, strlen(check_msg), 100);
        
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
    /* 简化版：移除了位置预判逻辑 */
    /* 中途停靠已由FSM_HandlePhotoSensor处理 */
    
    /* 超时保护：如果运行时间过长，可能有问题 */
    uint32_t elapsed = HAL_GetTick() - g_blackboard.state_enter_time;
    uint32_t max_time = abs(g_blackboard.target_floor - g_blackboard.current_floor) * 5000 + 5000;
    
    if (elapsed > max_time) {
        printf("[FSM] WARNING: Movement timeout! Stopping.\r\n");
        printf("  Current: F%d, Target: F%d, Time: %lu ms\r\n", 
               g_blackboard.current_floor, g_blackboard.target_floor, elapsed);
        
        /* 超时停止 */
        Blackboard_SetMotorCommand(MOTOR_CMD_EMERGENCY_STOP, 0);
        FSM_SendDirectionCommand(DIR_IDLE);
        Blackboard_SetState(STATE_IDLE);
        sprintf(g_blackboard.debug_msg, "TIMEOUT!");
    }
    
    /* 状态显示更新 */
    if (elapsed % 1000 == 0) {  // 每秒更新一次
        printf("[FSM] Moving: F%d -> F%d (%lu s)\r\n", 
               g_blackboard.current_floor, g_blackboard.target_floor, elapsed/1000);
    }
}


void FSM_StateDoorOperating(void) {
    uint32_t elapsed = HAL_GetTick() - door_operation_start;
    
    /* USART2调试 - 每500ms输出一次 */
    static uint32_t last_door_debug = 0;
    if (HAL_GetTick() - last_door_debug >= 500) {
        last_door_debug = HAL_GetTick();
        extern UART_HandleTypeDef huart2;
        char door_msg[80];
        sprintf(door_msg, "[U2-DOOR] Elapsed:%lu ms, State:%d\r\n", 
                elapsed, g_blackboard.door_state);
        HAL_UART_Transmit(&huart2, (uint8_t*)door_msg, strlen(door_msg), 100);
    }
    
    /* 真实门控版本 */
    if (!door_command_sent) {
        /* 发送开门命令 */
        FSM_SendDoorCommand(true);
        door_command_sent = true;
        g_blackboard.door_state = DOOR_OPENING;
        sprintf(g_blackboard.debug_msg, "Door opening");
    }
    else if (elapsed < 1500) {
        /* 等待开门（1.5秒） */
        g_blackboard.door_state = DOOR_OPENING;
    }
    else if (elapsed < 3500) {
        /* 保持开门（2秒） */
        g_blackboard.door_state = DOOR_OPEN;
        sprintf(g_blackboard.debug_msg, "Door open");
    }
    else if (elapsed < 3600) {
        /* 发送关门命令 */
        if (g_blackboard.door_state != DOOR_CLOSING) {
            FSM_SendDoorCommand(false);
            g_blackboard.door_state = DOOR_CLOSING;
            sprintf(g_blackboard.debug_msg, "Door closing");
        }
    }
    else if (elapsed < 5000) {
        /* 等待关门（1.5秒） */
        g_blackboard.door_state = DOOR_CLOSING;
    }
    else {
        /* 门操作完成 */
        printf("[FSM] Door operation completed\r\n");
        
        /* USART2调试 */
        extern UART_HandleTypeDef huart2;
        char complete_msg[60];
        sprintf(complete_msg, "[U2-DOOR] Completed after %lu ms\r\n", elapsed);
        HAL_UART_Transmit(&huart2, (uint8_t*)complete_msg, strlen(complete_msg), 100);
        
        /* 检查是否有继续的目标（中途停靠的情况） */
        if (g_blackboard.next_target_floor > 0 && 
            g_blackboard.next_target_floor != g_blackboard.current_floor) {
            /* 继续向原目标移动 */
            printf("[FSM] Continuing to floor %d\r\n", g_blackboard.next_target_floor);
            g_blackboard.target_floor = g_blackboard.next_target_floor;
            g_blackboard.next_target_floor = 0;
            
            /* 重新计算方向 */
            if (g_blackboard.target_floor > g_blackboard.current_floor) {
                g_blackboard.direction = DIR_UP;
            } else {
                g_blackboard.direction = DIR_DOWN;
            }
            
            /* 发送电机命令 */
            Blackboard_SetMotorCommand(MOTOR_CMD_MOVE_TO_FLOOR, g_blackboard.target_floor);
            FSM_SendDirectionCommand(g_blackboard.direction);
            
            /* 转回移动状态 */
            Blackboard_SetState(STATE_MOVING);
            sprintf(g_blackboard.debug_msg, "Go F%d", g_blackboard.target_floor);
        }
        else {
            /* 没有继续的目标，检查新的呼叫 */
            Blackboard_SetState(STATE_IDLE);
            sprintf(g_blackboard.debug_msg, "Idle");
            
            /* 推送到达事件，触发检查下一个呼叫 */
            Blackboard_PushEvent(EVENT_ARRIVED, g_blackboard.current_floor);
        }
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
    tx_buffer[1] = dir;  // 0=IDLE/STOP, 1=UP, 2=DOWN
    tx_buffer[2] = g_blackboard.current_floor;
    tx_buffer[3] = g_blackboard.target_floor;
    
    rs485_send_packet_dma(tx_buffer, 4);
    printf("[FSM] Direction set: %s, F%d->F%d\r\n", 
           dir == DIR_UP ? "UP" : dir == DIR_DOWN ? "DOWN" : "STOP",
           g_blackboard.current_floor, g_blackboard.target_floor);
}