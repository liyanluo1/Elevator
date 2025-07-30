#include "local_blackboard.h"
#include <string.h>
#include <stdio.h>

// 全局本地黑板实例
LocalBlackboard_t g_local_bb;

// 初始化本地黑板
void LocalBlackboard_Init(void) {
    // 清零所有内容
    memset(&g_local_bb, 0, sizeof(LocalBlackboard_t));
    
    // 设置默认值
    g_local_bb.current_floor = 1;     // 默认在1楼
    g_local_bb.target_floor = 1;      // 默认目标1楼
    g_local_bb.door = DOOR_CLOSED;    // 门默认关闭
    g_local_bb.state = ELEVATOR_IDLE; // 默认空闲状态
    g_local_bb.last_direction = DIR_IDLE;
    
    // 舵机默认位置（关门）
    g_local_bb.servo_position = 0;
    g_local_bb.servo_target_position = 0;
    g_local_bb.servo_busy = false;
    
    // 初始化事件队列
    g_local_bb.event_queue.head = 0;
    g_local_bb.event_queue.tail = 0;
    g_local_bb.event_queue.count = 0;
    
    // 清除同步标志
    g_local_bb.need_sync = false;
    g_local_bb.sync_fields = 0;
    
    // 时间戳初始化
    g_local_bb.timestamp = HAL_GetTick();
    g_local_bb.rs485_last_sync_time = HAL_GetTick();
}

// 重置本地黑板
void LocalBlackboard_Reset(void) {
    LocalBlackboard_Init();
}

// 推送事件到队列
bool LocalBlackboard_PushEvent(LocalEventType_t type, int data) {
    if (g_local_bb.event_queue.count >= LOCAL_EVENT_QUEUE_SIZE) {
        return false; // 队列满
    }
    
    LocalEvent_t* event = &g_local_bb.event_queue.events[g_local_bb.event_queue.tail];
    event->type = type;
    event->data = data;
    event->timestamp = HAL_GetTick();
    
    g_local_bb.event_queue.tail = (g_local_bb.event_queue.tail + 1) % LOCAL_EVENT_QUEUE_SIZE;
    g_local_bb.event_queue.count++;
    
    return true;
}

// 从队列弹出事件
bool LocalBlackboard_PopEvent(LocalEvent_t* event) {
    if (g_local_bb.event_queue.count == 0) {
        return false; // 队列空
    }
    
    *event = g_local_bb.event_queue.events[g_local_bb.event_queue.head];
    
    g_local_bb.event_queue.head = (g_local_bb.event_queue.head + 1) % LOCAL_EVENT_QUEUE_SIZE;
    g_local_bb.event_queue.count--;
    
    return true;
}

// 检查是否有事件
bool LocalBlackboard_HasEvents(void) {
    return g_local_bb.event_queue.count > 0;
}

// 清空事件队列
void LocalBlackboard_ClearEvents(void) {
    g_local_bb.event_queue.head = 0;
    g_local_bb.event_queue.tail = 0;
    g_local_bb.event_queue.count = 0;
}

// 设置状态
void LocalBlackboard_SetState(LocalElevatorState_t new_state) {
    g_local_bb.state = new_state;
    LocalBlackboard_MarkForSync(SYNC_FIELD_DOOR); // 状态改变通常伴随门状态变化
}

// 获取状态
LocalElevatorState_t LocalBlackboard_GetState(void) {
    return g_local_bb.state;
}

// 设置门状态
void LocalBlackboard_SetDoorState(DoorState_t state) {
    if (g_local_bb.door != state) {
        g_local_bb.door = state;
        LocalBlackboard_MarkForSync(SYNC_FIELD_DOOR);
        
        // 根据门状态推送相应事件
        if (state == DOOR_OPEN) {
            LocalBlackboard_PushEvent(EVENT_DOOR_OPENED, 0);
        } else if (state == DOOR_CLOSED) {
            LocalBlackboard_PushEvent(EVENT_DOOR_CLOSED, 0);
        }
    }
}

// 获取门状态
DoorState_t LocalBlackboard_GetDoorState(void) {
    return g_local_bb.door;
}

// 设置当前楼层
void LocalBlackboard_SetCurrentFloor(uint8_t floor) {
    if (g_local_bb.current_floor != floor) {
        g_local_bb.current_floor = floor;
        LocalBlackboard_MarkForSync(SYNC_FIELD_CURRENT_FLOOR);
        LocalBlackboard_PushEvent(EVENT_FLOOR_REACHED, floor);
    }
}

// 设置目标楼层
void LocalBlackboard_SetTargetFloor(uint8_t floor) {
    if (floor >= 1 && floor <= MAX_FLOORS) {
        if (g_local_bb.target_floor != floor) {
            g_local_bb.target_floor = floor;
            LocalBlackboard_MarkForSync(SYNC_FIELD_TARGET_FLOOR);
            LocalBlackboard_PushEvent(EVENT_TARGET_UPDATED, floor);
        }
    }
}

// 标记需要同步的字段
void LocalBlackboard_MarkForSync(uint32_t fields) {
    g_local_bb.sync_fields |= fields;
    g_local_bb.need_sync = true;
}

// 检查是否需要同步
bool LocalBlackboard_NeedsSync(void) {
    return g_local_bb.need_sync;
}

// 获取需要同步的字段
uint32_t LocalBlackboard_GetSyncFields(void) {
    return g_local_bb.sync_fields;
}

// 清除同步标志
void LocalBlackboard_ClearSyncFlags(void) {
    g_local_bb.sync_fields = 0;
    g_local_bb.need_sync = false;
    g_local_bb.rs485_last_sync_time = HAL_GetTick();
}

// 更新光电传感器状态
void LocalBlackboard_UpdateSensorState(bool triggered) {
    if (triggered && !g_local_bb.sensor_triggered) {
        // 上升沿检测
        g_local_bb.trigger_count++;
        g_local_bb.sensor_last_trigger = HAL_GetTick();
        LocalBlackboard_PushEvent(EVENT_SENSOR_TRIGGERED, g_local_bb.trigger_count);
    }
    g_local_bb.sensor_triggered = triggered;
}

// 根据方向和触发次数计算楼层
uint8_t LocalBlackboard_CalculateFloor(Direction_t dir, int trigger_count) {
    // 简化的楼层计算逻辑
    // 实际应用中需要考虑更多因素如起始位置、方向历史等
    uint8_t floor = 1;
    
    if (dir == DIR_UP) {
        floor = 1 + (trigger_count % MAX_FLOORS);
    } else if (dir == DIR_DOWN) {
        floor = MAX_FLOORS - (trigger_count % MAX_FLOORS);
    }
    
    // 确保楼层在有效范围内
    if (floor < 1) floor = 1;
    if (floor > MAX_FLOORS) floor = MAX_FLOORS;
    
    return floor;
}

// 设置键盘输入
void LocalBlackboard_SetKeyboardInput(uint8_t key) {
    g_local_bb.keyboard_buffer = key;
    g_local_bb.keyboard_new_input = true;
    LocalBlackboard_PushEvent(EVENT_KEYBOARD_INPUT, key);
}

// 检查是否有新的键盘输入
bool LocalBlackboard_HasNewKeyboardInput(void) {
    return g_local_bb.keyboard_new_input;
}

// 获取键盘输入
uint8_t LocalBlackboard_GetKeyboardInput(void) {
    g_local_bb.keyboard_new_input = false;
    return g_local_bb.keyboard_buffer;
}

// 调试输出
void LocalBlackboard_PrintStatus(void) {
    printf("=== Local Blackboard Status ===\n");
    printf("State: %d\n", g_local_bb.state);
    printf("Current Floor: %d\n", g_local_bb.current_floor);
    printf("Target Floor: %d\n", g_local_bb.target_floor);
    printf("Door State: %d\n", g_local_bb.door);
    printf("Servo Position: %d\n", g_local_bb.servo_position);
    printf("Error Code: %d\n", g_local_bb.error_code);
    printf("Event Queue Count: %d\n", g_local_bb.event_queue.count);
    printf("Need Sync: %s\n", g_local_bb.need_sync ? "Yes" : "No");
    printf("Sync Fields: 0x%08lX\n", g_local_bb.sync_fields);
    printf("===============================\n");
}