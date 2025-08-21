#include "local_blackboard.h"
#include "../RS485/rs485.h"
#include "../RS485/rs485_protocol.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"

/* 全局实例 */
LocalBlackboard_t g_local_bb;

/* 门控命令缓存 */
static LocalBB_DoorCommand_t pending_door_cmd = DOOR_CMD_NONE;

/* ==================== 初始化函数 ==================== */

void LocalBB_Init(void) {
    memset(&g_local_bb, 0, sizeof(LocalBlackboard_t));
    
    /* 初始状态 */
    g_local_bb.current_floor = 1;
    g_local_bb.expected_next_floor = 1;
    g_local_bb.direction = DIR_STOP;
    g_local_bb.door_state = LOCAL_DOOR_CLOSED;
    
    /* 事件队列 */
    g_local_bb.event_head = 0;
    g_local_bb.event_tail = 0;
    g_local_bb.event_count = 0;
    
    strcpy(g_local_bb.debug_msg, "LocalBB Init");
    printf("[LocalBB] Initialized\r\n");
}

void LocalBB_Reset(void) {
    LocalBB_Init();
}

/* ==================== 事件队列管理 ==================== */

static bool PushEvent(LocalEventType_t type, uint8_t data1, uint8_t data2, uint8_t data3) {
    /* 临界区保护 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    if (g_local_bb.event_count >= MAX_EVENT_QUEUE) {
        /* 队列满时丢弃最旧事件 */
        printf("[LocalBB] Event queue full! Dropping oldest\r\n");
        g_local_bb.event_head = (g_local_bb.event_head + 1) % MAX_EVENT_QUEUE;
        g_local_bb.event_count--;
    }
    
    LocalEvent_t* event = &g_local_bb.event_queue[g_local_bb.event_tail];
    event->type = type;
    event->data1 = data1;
    event->data2 = data2;
    event->data3 = data3;
    event->timestamp = HAL_GetTick();
    
    g_local_bb.event_tail = (g_local_bb.event_tail + 1) % MAX_EVENT_QUEUE;
    g_local_bb.event_count++;
    
    __set_PRIMASK(primask);
    return true;
}

static bool PopEvent(LocalEvent_t* event) {
    /* 临界区保护 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    if (g_local_bb.event_count == 0) {
        __set_PRIMASK(primask);
        return false;
    }
    
    *event = g_local_bb.event_queue[g_local_bb.event_head];
    g_local_bb.event_head = (g_local_bb.event_head + 1) % MAX_EVENT_QUEUE;
    g_local_bb.event_count--;
    
    __set_PRIMASK(primask);
    return true;
}

/* ==================== 事件输入接口 ==================== */

void LocalBB_AddCabinCall(uint8_t floor) {
    if (floor < 1 || floor > MAX_FLOORS) return;
    
    /* 防抖：如果短时间内重复按同一楼层，忽略 */
    uint32_t current_time = HAL_GetTick();
    if (floor == g_local_bb.last_sent_cabin_call && 
        (current_time - g_local_bb.last_cabin_call_time) < DEBOUNCE_TIME_MS) {
        printf("[LocalBB] Cabin call %d debounced\r\n", floor);
        return;
    }
    
    /* 关键修复：同层按钮特殊处理 */
    /* 每次都发送给Master，让Master决定是否开门 */
    /* 但要确保不会重复发送同一个呼叫 */
    
    PushEvent(LOCAL_EVENT_CABIN_CALL, floor, 0, 0);
    g_local_bb.cabin_call_count++;
    printf("[LocalBB] Cabin call queued: floor %d (current: %d)\r\n", floor, g_local_bb.current_floor);
}

void LocalBB_AddPhotoSensor(void) {
    /* 使用预期楼层 */
    uint8_t floor = g_local_bb.expected_next_floor;
    
    PushEvent(LOCAL_EVENT_PHOTO_SENSOR, floor, 0, 0);
    g_local_bb.photo_trigger_count++;
    printf("[LocalBB] Photo sensor queued: floor %d\r\n", floor);
    
    /* 更新当前楼层 */
    g_local_bb.current_floor = floor;
    
    /* 根据方向更新预期楼层 */
    if (g_local_bb.direction == DIR_UP) {
        g_local_bb.expected_next_floor = floor + 1;
        if (g_local_bb.expected_next_floor > MAX_FLOORS) {
            g_local_bb.expected_next_floor = MAX_FLOORS;
        }
    } else if (g_local_bb.direction == DIR_DOWN) {
        if (floor > 1) {
            g_local_bb.expected_next_floor = floor - 1;
        } else {
            g_local_bb.expected_next_floor = 1;
        }
    }
}

void LocalBB_SetDirection(uint8_t dir, uint8_t current, uint8_t target) {
    g_local_bb.direction = dir;
    g_local_bb.current_floor = current;
    g_local_bb.target_floor = target;
    
    /* 设置预期楼层 */
    if (dir == DIR_UP) {
        g_local_bb.expected_next_floor = current + 1;
    } else if (dir == DIR_DOWN && current > 1) {
        g_local_bb.expected_next_floor = current - 1;
    } else {
        g_local_bb.expected_next_floor = current;
    }
    
    PushEvent(LOCAL_EVENT_DIRECTION_SET, dir, current, target);
    printf("[LocalBB] Direction set: %s, %d->%d, expect %d\r\n", 
           dir == DIR_UP ? "UP" : dir == DIR_DOWN ? "DOWN" : "STOP",
           current, target, g_local_bb.expected_next_floor);
}

void LocalBB_AddDoorCommand(bool open) {
    PushEvent(LOCAL_EVENT_DOOR_CMD, open ? 1 : 0, 0, 0);
    g_local_bb.door_state = open ? LOCAL_DOOR_OPENING : LOCAL_DOOR_CLOSING;
    g_local_bb.door_cmd_time = HAL_GetTick();
    printf("[LocalBB] Door command queued: %s\r\n", open ? "OPEN" : "CLOSE");
}

/* ==================== 事件处理主函数 ==================== */

void LocalBB_Process(void) {
    LocalEvent_t event;
    uint32_t current_time = HAL_GetTick();
    
    while (PopEvent(&event)) {
        switch (event.type) {
            case LOCAL_EVENT_CABIN_CALL:
                /* 内呼总是发送，让Master决定如何处理 */
                /* 同层按钮需要立即响应，不能被防重复机制阻止 */
                {
                    /* 发送内呼命令 */
                    uint8_t tx_buffer[4];
                    tx_buffer[0] = CMD_CABIN_CALL;
                    tx_buffer[1] = event.data1;  // 楼层
                    tx_buffer[2] = 0;
                    tx_buffer[3] = 0;
                    
                    rs485_send_packet_dma(tx_buffer, 4);
                    g_local_bb.rs485_send_count++;
                    
                    g_local_bb.last_sent_cabin_call = event.data1;
                    g_local_bb.last_cabin_call_time = current_time;
                    
                    printf("[LocalBB] RS485 TX: Cabin call floor %d\r\n", event.data1);
                }
                break;
                
            case LOCAL_EVENT_PHOTO_SENSOR:
                /* 智能判断是否需要发送 */
                if (event.data1 != g_local_bb.last_sent_photo_floor ||
                    (current_time - g_local_bb.last_photo_send_time) > 200) {  // 光电允许更快
                    
                    /* 发送光电传感器触发 */
                    uint8_t tx_buffer[4];
                    tx_buffer[0] = CMD_PHOTO_SENSOR;
                    tx_buffer[1] = event.data1;  // 楼层
                    tx_buffer[2] = 0;
                    tx_buffer[3] = 0;
                    
                    rs485_send_packet_dma(tx_buffer, 4);
                    g_local_bb.rs485_send_count++;
                    
                    g_local_bb.last_sent_photo_floor = event.data1;
                    g_local_bb.last_photo_send_time = current_time;
                    
                    printf("[LocalBB] RS485 TX: Photo sensor floor %d\r\n", event.data1);
                }
                break;
                
            case LOCAL_EVENT_DIRECTION_SET:
                /* 方向设置只更新内部状态，不需要回传 */
                sprintf(g_local_bb.debug_msg, "Dir:%s F%d->%d", 
                        event.data1 == DIR_UP ? "UP" : 
                        event.data1 == DIR_DOWN ? "DN" : "ST",
                        event.data2, event.data3);
                break;
                
            case LOCAL_EVENT_DOOR_CMD:
                /* 设置门控命令 */
                pending_door_cmd = event.data1 ? DOOR_CMD_OPEN : DOOR_CMD_CLOSE;
                printf("[LocalBB] Door command ready: %s\r\n", 
                       pending_door_cmd == DOOR_CMD_OPEN ? "OPEN" : "CLOSE");
                break;
                
            default:
                break;
        }
    }
}

bool LocalBB_HasEvents(void) {
    return g_local_bb.event_count > 0;
}

/* ==================== 状态查询 ==================== */

uint8_t LocalBB_GetCurrentFloor(void) {
    return g_local_bb.current_floor;
}

uint8_t LocalBB_GetExpectedFloor(void) {
    return g_local_bb.expected_next_floor;
}

uint8_t LocalBB_GetDirection(void) {
    return g_local_bb.direction;
}

/* ==================== 调试输出 ==================== */

void LocalBB_PrintStatus(void) {
    printf("\r\n=== LocalBB Status ===\r\n");
    printf("Floor: %d (expect: %d)\r\n", 
           g_local_bb.current_floor, g_local_bb.expected_next_floor);
    printf("Direction: %s, Target: %d\r\n",
           g_local_bb.direction == DIR_UP ? "UP" : 
           g_local_bb.direction == DIR_DOWN ? "DOWN" : "STOP",
           g_local_bb.target_floor);
    printf("Events: %d in queue\r\n", g_local_bb.event_count);
    printf("Stats: Cabin=%lu, Photo=%lu, RS485=%lu\r\n",
           g_local_bb.cabin_call_count,
           g_local_bb.photo_trigger_count,
           g_local_bb.rs485_send_count);
    printf("Debug: %s\r\n", g_local_bb.debug_msg);
    printf("======================\r\n");
}

/* ==================== 门控命令接口 ==================== */

LocalBB_DoorCommand_t LocalBB_GetDoorCommand(void) {
    return pending_door_cmd;
}

void LocalBB_ClearDoorCommand(void) {
    pending_door_cmd = DOOR_CMD_NONE;
}

