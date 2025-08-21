#ifndef DOOR_CONTROL_H
#define DOOR_CONTROL_H

#include "servo.h"
#include <stdint.h>

// 门控状态枚举
typedef enum {
    DOOR_STATE_CLOSED,      // 门已关闭（222度位置）
    DOOR_STATE_OPENING,     // 正在开门
    DOOR_STATE_OPEN,        // 门已打开（0度位置）
    DOOR_STATE_CLOSING      // 正在关门
} DoorState_t;

// 门控位置定义（交换映射）
#define DOOR_POS_CLOSED    2526   // 关门位置：222度 (222*4096/360)
#define DOOR_POS_OPEN      0      // 开门位置：0度
#define DOOR_POS_TOLERANCE 50     // 位置容差（步数）

// 门控结构体
typedef struct {
    uint8_t servo_id;           // 舵机ID
    DoorState_t state;          // 当前状态
    uint16_t current_pos;       // 当前位置
    uint16_t target_pos;        // 目标位置
    uint32_t last_update_time;  // 上次更新时间
} DoorControl_t;

// 函数声明
void DoorControl_Init(DoorControl_t *door, uint8_t servo_id);
void DoorControl_Open(DoorControl_t *door);
void DoorControl_Close(DoorControl_t *door);
void DoorControl_Update(DoorControl_t *door);
DoorState_t DoorControl_GetState(DoorControl_t *door);
const char* DoorControl_GetStateName(DoorState_t state);
float DoorControl_GetAngle(DoorControl_t *door);
void DoorControl_PrintStatus(DoorControl_t *door);

#endif /* DOOR_CONTROL_H */