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
            /* 调试输出 */
            printf("[FSM_Process] Calling FSM_StateDoorOperating, door_command_sent=%d\r\n", door_command_sent);
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
    printf("[FSM] Button UP pressed at floor %d (current floor: %d, state: %s)\r\n", 
           floor, g_blackboard.current_floor, Blackboard_GetStateName(g_blackboard.state));
    
    /* 如果按的是当前楼层 */
    if (floor == g_blackboard.current_floor) {
        /* 如果电梯空闲，直接开门 */
        if (g_blackboard.state == STATE_IDLE) {
        printf("[FSM] Same floor UP button pressed, clearing any existing calls and opening door\r\n");
        
        /* 清除该楼层所有呼叫（避免重复执行） - 必须在状态切换前 */
        printf("[FSM] Before ClearCall: UP=%d, DOWN=%d, CABIN=%d\r\n", 
               g_blackboard.up_calls[floor], g_blackboard.down_calls[floor], g_blackboard.cabin_calls[floor]);
        Blackboard_ClearCall(floor);
        printf("[FSM] After ClearCall: UP=%d, DOWN=%d, CABIN=%d\r\n", 
               g_blackboard.up_calls[floor], g_blackboard.down_calls[floor], g_blackboard.cabin_calls[floor]);
        
        /* 强制重置文件级静态变量 - 关键修复 */
        door_command_sent = false;  // 必须在SetState之前重置
        
        /* 进入门操作状态 */
        Blackboard_SetState(STATE_DOOR_OPERATING);
        door_operation_start = HAL_GetTick();
        g_blackboard.door_state = DOOR_CLOSED;
        sprintf(g_blackboard.debug_msg, "SameFloor-UP F%d", floor);
        
        printf("[FSM] Door operation started, door_command_sent forcefully reset to %d\r\n", door_command_sent);
        printf("[FSM] door_operation_start set to %lu\r\n", door_operation_start);
        printf("[FSM] Current state: %s\r\n", Blackboard_GetStateName(g_blackboard.state));
        }
        /* 如果电梯已经在门操作状态，忽略此次按钮 */
        else if (g_blackboard.state == STATE_DOOR_OPERATING) {
            printf("[FSM] Same floor button pressed but door already operating, ignoring\r\n");
        }
        /* 如果电梯在移动或其他状态，不处理同层按钮 */
        else {
            printf("[FSM] Same floor button pressed but elevator busy (state=%s), ignoring\r\n",
                   Blackboard_GetStateName(g_blackboard.state));
        }
        return;  // 同层按钮总是直接返回，不添加呼叫
    }
    
    /* 不是同层，添加呼叫 */
    Blackboard_AddUpCall(floor);
    
    /* 如果电梯空闲，触发状态机检查 */
    if (g_blackboard.state == STATE_IDLE) {
        /* 直接触发响应，不再推送事件 */
        FSM_CheckAndStartMovement();
    }
}

void FSM_HandleButtonDown(uint8_t floor) {
    printf("[FSM] Button DOWN pressed at floor %d (current floor: %d, state: %s)\r\n", 
           floor, g_blackboard.current_floor, Blackboard_GetStateName(g_blackboard.state));
    
    /* 如果按的是当前楼层 */
    if (floor == g_blackboard.current_floor) {
        /* 如果电梯空闲，直接开门 */
        if (g_blackboard.state == STATE_IDLE) {
        printf("[FSM] Same floor DOWN button pressed, clearing any existing calls and opening door\r\n");
        
        /* 清除该楼层所有呼叫（避免重复执行） - 必须在状态切换前 */
        Blackboard_ClearCall(floor);
        
        /* 强制重置文件级静态变量 - 关键修复 */
        door_command_sent = false;  // 必须在SetState之前重置
        
        /* 进入门操作状态 */
        Blackboard_SetState(STATE_DOOR_OPERATING);
        door_operation_start = HAL_GetTick();
        g_blackboard.door_state = DOOR_CLOSED;
        sprintf(g_blackboard.debug_msg, "SameFloor-DN F%d", floor);
        
        printf("[FSM] Door operation started, door_command_sent forcefully reset to %d\r\n", door_command_sent);
        printf("[FSM] door_operation_start set to %lu\r\n", door_operation_start);
        }
        /* 如果电梯已经在门操作状态，忽略此次按钮 */
        else if (g_blackboard.state == STATE_DOOR_OPERATING) {
            printf("[FSM] Same floor button pressed but door already operating, ignoring\r\n");
        }
        /* 如果电梯在移动或其他状态，不处理同层按钮 */
        else {
            printf("[FSM] Same floor button pressed but elevator busy (state=%s), ignoring\r\n",
                   Blackboard_GetStateName(g_blackboard.state));
        }
        return;  // 同层按钮总是直接返回，不添加呼叫
    }
    
    /* 不是同层或不是空闲状态，添加呼叫 */
    Blackboard_AddDownCall(floor);
    
    /* 如果电梯空闲，触发状态机检查 */
    if (g_blackboard.state == STATE_IDLE) {
        FSM_CheckAndStartMovement();
    }
}

void FSM_HandleCabinCall(uint8_t floor) {
    printf("[FSM] Cabin call for floor %d (current floor: %d, state: %s)\r\n", 
           floor, g_blackboard.current_floor, Blackboard_GetStateName(g_blackboard.state));
    
    /* 如果按的是当前楼层，优先处理开门（不管什么状态） */
    if (floor == g_blackboard.current_floor) {
        /* 检查是否已经在门操作状态 */
        if (g_blackboard.state == STATE_DOOR_OPERATING) {
            printf("[FSM] Already in door operation, ignoring same floor call\r\n");
            return;  // 已经在开门，忽略
        }
        
        /* 如果在移动状态，不处理同层呼叫 */
        if (g_blackboard.state == STATE_MOVING) {
            printf("[FSM] In moving state, adding cabin call for later\r\n");
            Blackboard_AddCabinCall(floor);
            return;
        }
        
        /* 只在IDLE状态下处理同层内呼 - 与外呼逻辑一致 */
        if (g_blackboard.state == STATE_IDLE) {
            printf("[FSM] Same floor cabin call in IDLE, clearing calls and opening door\r\n");
            
            /* 与外呼逻辑一致：先清除呼叫，再开门 */
            Blackboard_ClearCall(floor);
            
            /* 强制重置文件级静态变量 */
            door_command_sent = false;
            
            /* 进入门操作状态 */
            Blackboard_SetState(STATE_DOOR_OPERATING);
            door_operation_start = HAL_GetTick();
            g_blackboard.door_state = DOOR_CLOSED;
            sprintf(g_blackboard.debug_msg, "SameFloor-CAB F%d", floor);
            
            printf("[FSM] Door operation started\r\n");
            return;  // 同层开门后直接返回
        }
        
        /* 其他状态不处理同层内呼 */
        printf("[FSM] Same floor cabin call but not in IDLE (state=%s), ignoring\r\n",
               Blackboard_GetStateName(g_blackboard.state));
        return;
    }
    
    /* 不是同层，添加呼叫 */
    Blackboard_AddCabinCall(floor);
    
    /* 如果电梯空闲，触发状态机检查 */
    if (g_blackboard.state == STATE_IDLE) {
        FSM_CheckAndStartMovement();
    }
}

void FSM_HandlePhotoSensor(uint8_t floor) {
    printf("\r\n******** FSM PHOTO SENSOR HANDLER ********\r\n");
    printf("[FSM] Photo sensor triggered at floor %d (state=%s)\r\n", 
           floor, Blackboard_GetStateName(g_blackboard.state));
    printf("[FSM] Target floor: %d, Current floor: %d -> %d\r\n", 
           g_blackboard.target_floor, g_blackboard.current_floor, floor);
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
            printf("[FSM] *** TARGET FLOOR REACHED! ***\r\n");
            printf("[FSM] Stopping at floor %d\r\n", floor);
            
            /* 立即停止电机 */
            Blackboard_SetMotorCommand(MOTOR_CMD_STOP, 0);
            
            /* 不要立即清除，等门操作完成后清除 */
            printf("[FSM] Will clear calls at target floor %d after door operation\r\n", floor);
            /* Blackboard_ClearCall(floor); -- 移到门操作完成后 */
            
            /* 发送停止方向命令 */
            FSM_SendDirectionCommand(DIR_STOP);
            
            /* 转到门操作状态 */
            printf("[FSM] >>> TRANSITIONING TO DOOR_OPERATING STATE <<<\r\n");
            printf("[FSM] door_state initial value: %d\r\n", g_blackboard.door_state);
            Blackboard_SetState(STATE_DOOR_OPERATING);
            door_operation_start = HAL_GetTick();  // 记录开始时间
            door_command_sent = false;
            g_blackboard.door_state = DOOR_CLOSED;  // 重要：初始化门状态为关闭
            sprintf(g_blackboard.debug_msg, "Door F%d", floor);
            printf("[FSM] door_state reset to CLOSED, door_operation_start=%lu\r\n", door_operation_start);
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
                
                /* 不要立即清除，等门操作完成后清除 */
                printf("[FSM] Will clear calls at intermediate floor %d after door operation\r\n", floor);
                /* 原来的清除逻辑移到门操作完成后 */
                
                /* 转到门操作状态（非阻塞） */
                printf("[FSM] Starting intermediate door operation\r\n");
                
                /* 记住要继续去的目标 */
                g_blackboard.next_target_floor = g_blackboard.target_floor;
                
                /* 转到门操作状态 */
                Blackboard_SetState(STATE_DOOR_OPERATING);
                door_operation_start = HAL_GetTick();
                door_command_sent = false;
                g_blackboard.door_state = DOOR_CLOSED;  // 重要：初始化门状态为关闭
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
    
    printf("*******************************************\r\n\r\n");
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
            /* 就在当前楼层，需要进行门操作 */
            printf("[FSM] Already at floor %d, starting door operation\r\n", g_blackboard.current_floor);
            sprintf(check_msg, "[U2-CHECK] Already at F%d, door op\r\n", g_blackboard.current_floor);
            HAL_UART_Transmit(&huart2, (uint8_t*)check_msg, strlen(check_msg), 100);
            
            /* 不要立即清除呼叫，等门操作完成后清除 */
            /* Blackboard_ClearCall(g_blackboard.current_floor); -- 移除这行 */
            
            /* 直接进入门操作状态 */
            Blackboard_SetState(STATE_DOOR_OPERATING);
            door_operation_start = HAL_GetTick();
            door_command_sent = false;
            g_blackboard.door_state = DOOR_CLOSED;  // 重要：初始化门状态为关闭
            sprintf(g_blackboard.debug_msg, "Door@F%d", g_blackboard.current_floor);
        }
    }
}

void FSM_StateMoving(void) {
    /* 简化版：移除了位置预判逻辑 */
    /* 中途停靠已由FSM_HandlePhotoSensor处理 */
    
    /* 增强的超时保护 */
    uint32_t elapsed = HAL_GetTick() - g_blackboard.state_enter_time;
    uint32_t floors_to_travel = abs(g_blackboard.target_floor - g_blackboard.current_floor);
    uint32_t max_time = floors_to_travel * 5000 + 10000;  // 每层5秒 + 10秒余量
    
    if (elapsed > max_time) {
        printf("[FSM] ERROR: Movement timeout! Emergency stop\r\n");
        printf("  Current: F%d, Target: F%d, Time: %lu ms, Max: %lu ms\r\n", 
               g_blackboard.current_floor, g_blackboard.target_floor, elapsed, max_time);
        
        /* 紧急停止 */
        Blackboard_SetMotorCommand(MOTOR_CMD_EMERGENCY_STOP, 0);
        FSM_SendDirectionCommand(DIR_STOP);
        
        /* 清除目标楼层的呼叫，避免重复尝试 */
        if (g_blackboard.target_floor > 0 && g_blackboard.target_floor <= MAX_FLOORS) {
            Blackboard_ClearCall(g_blackboard.target_floor);
            printf("[FSM] Cleared call at target floor %d due to timeout\r\n", g_blackboard.target_floor);
        }
        
        Blackboard_SetState(STATE_IDLE);
        sprintf(g_blackboard.debug_msg, "MOV TIMEOUT!");
        
        /* 推送超时事件 */
        Blackboard_PushEvent(EVENT_TIMEOUT, g_blackboard.current_floor);
    }
    
    /* 状态显示更新 */
    if (elapsed % 1000 == 0) {  // 每秒更新一次
        printf("[FSM] Moving: F%d -> F%d (%lu s)\r\n", 
               g_blackboard.current_floor, g_blackboard.target_floor, elapsed/1000);
    }
}


void FSM_StateDoorOperating(void) {
    uint32_t elapsed = HAL_GetTick() - door_operation_start;
    static uint32_t last_state_enter_time = 0;
    static uint32_t door_open_time = 0;  // 门完全打开的时间
    static bool close_command_sent = false;
    static bool first_call = true;  // 标记是否第一次调用
    
    /* 调试输出：每次进入函数时的状态 */
    static uint32_t call_count = 0;
    call_count++;
    if (call_count % 10 == 1) {  // 每10次调用打印一次
        printf("[DOOR_DEBUG] Call#%lu: elapsed=%lu, door_cmd_sent=%d, last_enter=%lu, curr_enter=%lu\r\n",
               call_count, elapsed, door_command_sent, last_state_enter_time, g_blackboard.state_enter_time);
    }
    
    /* 超时保护 - 最长15秒 */
    #define DOOR_OPERATION_TIMEOUT_MS 15000
    if (elapsed > DOOR_OPERATION_TIMEOUT_MS) {
        printf("[FSM] ERROR: Door operation timeout! Force completing\r\n");
        /* 强制完成门操作 */
        g_blackboard.door_state = DOOR_CLOSED;
        door_command_sent = false;
        Blackboard_SetState(STATE_IDLE);
        sprintf(g_blackboard.debug_msg, "Door timeout!");
        /* 推送到达事件，继续处理其他呼叫 */
        Blackboard_PushEvent(EVENT_ARRIVED, g_blackboard.current_floor);
        return;
    }
    
    /* 检测状态变化，重置静态变量 - 修复：添加首次调用检测 */
    if (first_call || g_blackboard.state_enter_time != last_state_enter_time) {
        if (first_call) {
            printf("\r\n[FSM] === FIRST CALL TO STATE_DOOR_OPERATING ===\r\n");
            first_call = false;
        } else {
            printf("\r\n[FSM] === STATE CHANGE DETECTED ===\r\n");
        }
        printf("[FSM] Previous enter time: %lu, New enter time: %lu\r\n", 
               last_state_enter_time, g_blackboard.state_enter_time);
        
        last_state_enter_time = g_blackboard.state_enter_time;
        
        /* 重置所有静态变量（包括文件级静态变量） */
        door_open_time = 0;
        close_command_sent = false;
        door_command_sent = false;  // 重要：重置文件级静态变量
        
        printf("[FSM] === ENTERED STATE_DOOR_OPERATING ===\r\n");
        printf("[FSM] Entry tick: %lu, door_state: %d\r\n", last_state_enter_time, g_blackboard.door_state);
        printf("[FSM] door_command_sent reset to false\r\n");
        printf("[FSM] Static vars reset: door_open_time=0, close_command_sent=false\r\n");
        printf("[FSM] door_operation_start = %lu\r\n", door_operation_start);
        printf("=====================================\r\n");
    }
    
    /* USART2调试 - 每500ms输出一次 */
    static uint32_t last_door_debug = 0;
    if (HAL_GetTick() - last_door_debug >= 500) {
        last_door_debug = HAL_GetTick();
        extern UART_HandleTypeDef huart2;
        char door_msg[80];
        const char* state_str[] = {"CLOSED", "OPENING", "OPEN", "CLOSING"};
        sprintf(door_msg, "[U2-DOOR] Elapsed:%lu ms, State:%s\r\n", 
                elapsed, state_str[g_blackboard.door_state]);
        HAL_UART_Transmit(&huart2, (uint8_t*)door_msg, strlen(door_msg), 100);
        
        /* 同时打印到USART1 */
        printf("[DOOR_OP] Elapsed: %lu ms, door_state: %s, cmd_sent: %d\r\n",
               elapsed, state_str[g_blackboard.door_state], door_command_sent);
    }
    
    /* TIME BASED控制：纯时间控制，不依赖反馈 */
    if (!door_command_sent) {
        /* 发送开门命令 */
        printf("[FSM] >>> SENDING DOOR OPEN COMMAND <<<\r\n");
        printf("[FSM] door_command_sent is false, sending open command now\r\n");
        FSM_SendDoorCommand(true);
        door_command_sent = true;
        close_command_sent = false;
        sprintf(g_blackboard.debug_msg, "Sent open cmd");
        printf("[FSM] Door open command sent at tick %lu\r\n", HAL_GetTick());
        printf("[FSM] door_command_sent now set to true\r\n");
        
        /* 假设门正在开启 */
        g_blackboard.door_state = DOOR_OPENING;
    }
    else if (elapsed < 2000) {
        /* 等待2秒让门打开 */
        sprintf(g_blackboard.debug_msg, "Door opening");
        // TIME BASED模式 - 不检查反馈
    }
    else if (elapsed >= 2000 && elapsed < 5000) {
        /* 门已完全打开 - 保持开门3秒 */
        if (door_open_time == 0) {
            door_open_time = HAL_GetTick();
            printf("[FSM] Door fully opened (time-based)\r\n");
            g_blackboard.door_state = DOOR_OPEN;  // 假设门已打开
        }
        
        sprintf(g_blackboard.debug_msg, "Door open");
    }
    else if (elapsed >= 5000 && !close_command_sent) {
        /* 发送关门命令 */
        printf("[FSM] >>> SENDING DOOR CLOSE COMMAND <<<\r\n");
        FSM_SendDoorCommand(false);
        close_command_sent = true;
        g_blackboard.door_state = DOOR_CLOSING;  // 假设门正在关闭
        sprintf(g_blackboard.debug_msg, "Sent close cmd");
    }
    else if (elapsed >= 5000 && close_command_sent && elapsed < 8000) {
        /* 等待门关闭（3秒时间） */
        sprintf(g_blackboard.debug_msg, "Door closing");
        // TIME BASED模式 - 不检查反馈
    }
    else if (elapsed >= 8000) {
        /* 门已完全关闭，操作完成 */
        printf("[FSM] Door operation completed (time-based, elapsed=%lu ms)\r\n", elapsed);
        g_blackboard.door_state = DOOR_CLOSED;  // 确保状态为关闭
        
        /* 重置门控制命令标志 */
        door_command_sent = false;
        
        /* 现在清除当前楼层的呼叫（门操作完成后） */
        printf("[FSM] Clearing calls at current floor %d after door operation\r\n", g_blackboard.current_floor);
        Blackboard_ClearCall(g_blackboard.current_floor);
        
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