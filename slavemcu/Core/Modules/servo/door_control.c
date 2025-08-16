#include "door_control.h"
#include <stdio.h>
#include "stm32f1xx_hal.h"

/**
 * @brief 初始化门控系统
 */
void DoorControl_Init(DoorControl_t *door, uint8_t servo_id) {
    door->servo_id = servo_id;
    door->current_pos = servo_get_position(servo_id);
    door->target_pos = door->current_pos;
    door->last_update_time = HAL_GetTick();
    
    // 根据当前位置判断初始状态
    if (door->current_pos < (DOOR_POS_CLOSED + DOOR_POS_TOLERANCE)) {
        door->state = DOOR_STATE_CLOSED;
        printf("[DOOR] Initial state: CLOSED (pos=%u, angle=%.1f°)\r\n", 
               door->current_pos, door->current_pos * 0.087890625);
    } else if (door->current_pos > (DOOR_POS_OPEN - DOOR_POS_TOLERANCE)) {
        door->state = DOOR_STATE_OPEN;
        printf("[DOOR] Initial state: OPEN (pos=%u, angle=%.1f°)\r\n", 
               door->current_pos, door->current_pos * 0.087890625);
    } else {
        // 中间位置，默认关门
        door->state = DOOR_STATE_CLOSING;
        door->target_pos = DOOR_POS_CLOSED;
        servo_set_position(door->servo_id, door->target_pos);
        printf("[DOOR] Initial state: INTERMEDIATE, closing to 222° (current pos=%u, angle=%.1f°)\r\n", 
               door->current_pos, door->current_pos * 0.087890625);
    }
}

/**
 * @brief 开门命令
 */
void DoorControl_Open(DoorControl_t *door) {
    if (door->state == DOOR_STATE_CLOSED || door->state == DOOR_STATE_CLOSING) {
        door->state = DOOR_STATE_OPENING;
        door->target_pos = DOOR_POS_OPEN;
        servo_set_position(door->servo_id, door->target_pos);
        printf("[DOOR] Command: OPEN (moving to 0°, target=%u)\r\n", door->target_pos);
    } else if (door->state == DOOR_STATE_OPEN) {
        printf("[DOOR] Already OPEN\r\n");
    }
}

/**
 * @brief 关门命令
 */
void DoorControl_Close(DoorControl_t *door) {
    if (door->state == DOOR_STATE_OPEN || door->state == DOOR_STATE_OPENING) {
        door->state = DOOR_STATE_CLOSING;
        door->target_pos = DOOR_POS_CLOSED;
        servo_set_position(door->servo_id, door->target_pos);
        printf("[DOOR] Command: CLOSE (moving to 222°, target=%u)\r\n", door->target_pos);
    } else if (door->state == DOOR_STATE_CLOSED) {
        printf("[DOOR] Already CLOSED\r\n");
    }
}

/**
 * @brief 更新门控状态
 */
void DoorControl_Update(DoorControl_t *door) {
    uint32_t current_time = HAL_GetTick();
    
    // 每50ms更新一次
    if (current_time - door->last_update_time < 50) {
        return;
    }
    door->last_update_time = current_time;
    
    // 读取当前位置
    door->current_pos = servo_get_position(door->servo_id);
    
    // 根据当前状态更新
    switch (door->state) {
        case DOOR_STATE_OPENING:
            // 检查是否到达开门位置
            if (door->current_pos >= (DOOR_POS_OPEN - DOOR_POS_TOLERANCE)) {
                door->state = DOOR_STATE_OPEN;
                printf("[DOOR] State changed: OPEN (pos=%u, angle=%.1f°)\r\n", 
                       door->current_pos, door->current_pos * 0.087890625);
            }
            break;
            
        case DOOR_STATE_CLOSING:
            // 检查是否到达关门位置
            if (door->current_pos <= (DOOR_POS_CLOSED + DOOR_POS_TOLERANCE)) {
                door->state = DOOR_STATE_CLOSED;
                printf("[DOOR] State changed: CLOSED (pos=%u, angle=%.1f°)\r\n", 
                       door->current_pos, door->current_pos * 0.087890625);
            }
            break;
            
        case DOOR_STATE_OPEN:
        case DOOR_STATE_CLOSED:
            // 稳定状态，不需要做什么
            break;
    }
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
        case DOOR_STATE_CLOSED:   return "CLOSED (222°)";
        case DOOR_STATE_OPENING:  return "OPENING";
        case DOOR_STATE_OPEN:     return "OPEN (0°)";
        case DOOR_STATE_CLOSING:  return "CLOSING";
        default:                  return "UNKNOWN";
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