#include "door_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

/* 私有变量 */
static uint32_t update_count = 0;

/**
 * @brief 初始化门控系统 - 4状态机版本
 */
void DoorControl_Init(DoorControl_t *door, uint8_t servo_id) {
    printf("[DOOR] Initializing door control system (4-State FSM)...\r\n");
    
    door->servo_id = servo_id;
    
    /* 初始化舵机 */
    servo_set_speed(servo_id, 500);
    servo_set_torque_enable(servo_id, 1);
    HAL_Delay(100);
    
    /* 读取当前位置 */
    uint8_t rx_data[2];
    servo_read_reg(servo_id, 0x38, 2, rx_data);
    door->current_pos = rx_data[0] | (rx_data[1] << 8);
    
    /* 读取移动标志 */
    uint8_t moving_flag;
    servo_read_reg(servo_id, 0x42, 1, &moving_flag);
    
    printf("[DOOR] Initial position: %u (target OPEN=%u, CLOSED=%u)\r\n", 
           door->current_pos, DOOR_POS_OPEN, DOOR_POS_CLOSED);
    printf("[DOOR] Moving flag: %u\r\n", moving_flag);
    printf("[DOOR] Distance to OPEN: %d, Distance to CLOSED: %d\r\n",
           abs((int)door->current_pos - DOOR_POS_OPEN),
           abs((int)door->current_pos - DOOR_POS_CLOSED));
    
    /* 处理位置环绕：4093-4095接近0 */
    if (door->current_pos > 4090) {
        printf("[DOOR] Position wrap-around detected (%u -> 0)\r\n", door->current_pos);
        door->current_pos = 0;  // 修正为0
    }
    
    /* 判断初始状态 */
    if (moving_flag) {
        /* 正在移动，无法确定方向，默认认为在关门 */
        printf("[DOOR] Motor is moving at startup, assuming closing...\r\n");
        door->state = DOOR_STATE_CLOSING;
        door->target_pos = DOOR_POS_CLOSED;
    } else {
        /* 静止状态，默认认为是关闭的 */
        door->state = DOOR_STATE_CLOSED;
        door->target_pos = DOOR_POS_CLOSED;
        printf("[DOOR] Motor is stopped, assuming door is closed\r\n");
        
        /* 为了安全，发送一次关门命令确保门关闭 */
        servo_set_position(servo_id, DOOR_POS_CLOSED);
    }
    
    door->last_update_time = HAL_GetTick();
    printf("[DOOR] Initial state: %s\r\n", DoorControl_GetStateName(door->state));
    printf("[DOOR] Initialization complete\r\n");
}

/**
 * @brief 开门命令
 */
void DoorControl_Open(DoorControl_t *door) {
    printf("\r\n[DOOR_DEBUG] ===== OPEN COMMAND =====\r\n");
    printf("[DOOR_DEBUG] Current state: %s, Current pos: %u\r\n", 
           DoorControl_GetStateName(door->state), door->current_pos);
    
    if (door->state == DOOR_STATE_CLOSED || door->state == DOOR_STATE_CLOSING) {
        printf("[CMD] Sending OPEN command (target: %u)\r\n", DOOR_POS_OPEN);
        printf("[DOOR_DEBUG] Position change: %u -> %u (delta: %d)\r\n", 
               door->current_pos, DOOR_POS_OPEN, 
               (int)DOOR_POS_OPEN - (int)door->current_pos);
        
        /* 添加小延时确保之前的通信完成 */
        HAL_Delay(5);
        servo_set_position(door->servo_id, DOOR_POS_OPEN);
        HAL_Delay(5);  /* 给舵机时间处理命令 */
        
        door->target_pos = DOOR_POS_OPEN;
        door->state = DOOR_STATE_OPENING;
        
        printf("[DOOR_DEBUG] State changed to OPENING at tick %lu\r\n", HAL_GetTick());
    } else {
        printf("[CMD] Cannot open - current state: %s\r\n", 
               DoorControl_GetStateName(door->state));
    }
}

/**
 * @brief 关门命令
 */
void DoorControl_Close(DoorControl_t *door) {
    printf("\r\n[DOOR_DEBUG] ===== CLOSE COMMAND =====\r\n");
    printf("[DOOR_DEBUG] Current state: %s, Current pos: %u\r\n", 
           DoorControl_GetStateName(door->state), door->current_pos);
    
    if (door->state == DOOR_STATE_OPEN || door->state == DOOR_STATE_OPENING) {
        printf("[CMD] Sending CLOSE command (target: %u)\r\n", DOOR_POS_CLOSED);
        printf("[DOOR_DEBUG] Position change: %u -> %u (delta: %d)\r\n", 
               door->current_pos, DOOR_POS_CLOSED, 
               (int)DOOR_POS_CLOSED - (int)door->current_pos);
        
        /* 添加小延时确保之前的通信完成 */
        HAL_Delay(5);
        servo_set_position(door->servo_id, DOOR_POS_CLOSED);
        HAL_Delay(5);  /* 给舵机时间处理命令 */
        
        door->target_pos = DOOR_POS_CLOSED;
        door->state = DOOR_STATE_CLOSING;
        
        printf("[DOOR_DEBUG] State changed to CLOSING at tick %lu\r\n", HAL_GetTick());
    } else {
        printf("[CMD] Cannot close - current state: %s\r\n", 
               DoorControl_GetStateName(door->state));
    }
}

/**
 * @brief 更新门控状态（核心状态机 - 使用移动标志）
 */
void DoorControl_Update(DoorControl_t *door) {
    update_count++;
    
    /* 读取当前位置 - 带重试和验证 */
    uint8_t rx_data[2];
    uint16_t new_pos = 0;
    bool pos_valid = false;
    
    for (int retry = 0; retry < 3; retry++) {
        if (servo_read_reg(door->servo_id, 0x38, 2, rx_data) == 1) {  /* 返回1表示成功 */
            new_pos = rx_data[0] | (rx_data[1] << 8);
            
            /* 验证位置值是否合理（0-4095范围） */
            if (new_pos <= 4095) {
                pos_valid = true;
                break;
            }
        }
        HAL_Delay(2);  /* 短暂延时后重试 */
    }
    
    uint16_t old_pos = door->current_pos;
    if (pos_valid) {
        door->current_pos = new_pos;
        
        /* 处理位置环绕 */
        if (door->current_pos > 4090) {
            door->current_pos = 0;
        }
    } else {
        printf("[DOOR] Warning: Failed to read position, keeping old value %u\r\n", old_pos);
    }
    
    /* 读取移动标志 - 带重试和验证 */
    uint8_t moving_flag = 0;
    bool flag_valid = false;
    
    for (int retry = 0; retry < 3; retry++) {
        uint8_t temp_flag;
        if (servo_read_reg(door->servo_id, 0x42, 1, &temp_flag) == 1) {  /* 返回1表示成功 */
            /* 验证标志值（应该是0或1） */
            if (temp_flag <= 1) {
                moving_flag = temp_flag;
                flag_valid = true;
                break;
            } else if (temp_flag & 0x01) {
                /* 如果是其他位被设置，只看最低位 */
                moving_flag = 1;
                flag_valid = true;
                printf("[DOOR] Moving flag raw value: 0x%02X, using bit 0: %d\r\n", temp_flag, moving_flag);
                break;
            }
        }
        HAL_Delay(2);  /* 短暂延时后重试 */
    }
    
    if (!flag_valid) {
        printf("[DOOR] Warning: Failed to read moving flag, assuming stopped\r\n");
        moving_flag = 0;
    }
    
    /* 调试输出 - 每500ms或状态变化时 */
    static uint32_t last_debug_time = 0;
    static uint8_t last_moving_flag = 0xFF;
    uint32_t now = HAL_GetTick();
    
    if ((now - last_debug_time >= 500) || 
        (moving_flag != last_moving_flag) ||
        (abs((int)door->current_pos - (int)old_pos) > 100)) {
        
        printf("[UPDATE] T=%lu, State=%s, Pos=%u, Target=%u, Moving=%d, dPos=%d\r\n",
               now, DoorControl_GetStateName(door->state),
               door->current_pos, door->target_pos, 
               moving_flag, (int)door->current_pos - (int)old_pos);
        
        last_debug_time = now;
        last_moving_flag = moving_flag;
    }
    
    /* 保存旧状态 */
    DoorState_t old_state = door->state;
    
    /* 状态机逻辑 */
    switch (door->state) {
        case DOOR_STATE_CLOSED:
            /* 关闭状态 - 等待开门命令 */
            // 命令在外部发送
            break;
            
        case DOOR_STATE_OPENING:
            /* 正在开门 - 检查是否停止 */
            printf("[OPENING_CHECK] Moving=%d, Pos=%u, Target=%u, Time since cmd=%lu ms\r\n",
                   moving_flag, door->current_pos, door->target_pos,
                   HAL_GetTick() - door->last_update_time);
            
            if (!moving_flag) {
                /* 舵机停止了，假设门已开 */
                door->state = DOOR_STATE_OPEN;
                printf("[STATE] OPENING -> OPEN (motor stopped at pos %u)\r\n", door->current_pos);
                printf("[DOOR_DEBUG] Door OPEN detected at tick %lu\r\n", HAL_GetTick());
            }
            break;
            
        case DOOR_STATE_OPEN:
            /* 打开状态 - 等待关门命令 */
            // 命令在外部发送
            break;
            
        case DOOR_STATE_CLOSING:
            /* 正在关门 - 检查是否停止 */
            printf("[CLOSING_CHECK] Moving=%d, Pos=%u, Target=%u, Time since cmd=%lu ms\r\n",
                   moving_flag, door->current_pos, door->target_pos,
                   HAL_GetTick() - door->last_update_time);
            
            if (!moving_flag) {
                /* 舵机停止了，假设门已关 */
                door->state = DOOR_STATE_CLOSED;
                printf("[STATE] CLOSING -> CLOSED (motor stopped at pos %u)\r\n", door->current_pos);
                printf("[DOOR_DEBUG] Door CLOSED detected at tick %lu\r\n", HAL_GetTick());
            }
            break;
    }
    
    /* 状态变化时输出 */
    if (door->state != old_state) {
        printf("\r\n[TRANSITION] %s -> %s\r\n", 
               DoorControl_GetStateName(old_state),
               DoorControl_GetStateName(door->state));
    }
    
    door->last_update_time = HAL_GetTick();
}

/**
 * @brief 获取当前状态
 */
DoorState_t DoorControl_GetState(DoorControl_t *door) {
    return door->state;
}

/**
 * @brief 获取状态名称字符串
 */
const char* DoorControl_GetStateName(DoorState_t state) {
    switch (state) {
        case DOOR_STATE_CLOSED:  return "CLOSED";
        case DOOR_STATE_OPENING: return "OPENING";
        case DOOR_STATE_OPEN:    return "OPEN";
        case DOOR_STATE_CLOSING: return "CLOSING";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 获取当前角度
 */
float DoorControl_GetAngle(DoorControl_t *door) {
    return door->current_pos * 0.087890625f;
}

/**
 * @brief 打印状态信息
 */
void DoorControl_PrintStatus(DoorControl_t *door) {
    printf("[DOOR STATUS] State: %s, Pos: %u, Angle: %.1f°, Target: %u\r\n",
           DoorControl_GetStateName(door->state),
           door->current_pos,
           DoorControl_GetAngle(door),
           door->target_pos);
}