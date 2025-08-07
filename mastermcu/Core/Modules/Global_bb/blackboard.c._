#include "blackboard.h"
#include <string.h>
#include <stdio.h>

// 全局黑板实例
GlobalBlackboard_t g_blackboard;

// 初始化黑板
void Blackboard_Init(void) {
    memset(&g_blackboard, 0, sizeof(GlobalBlackboard_t));
    
    // 设置默认参数
    g_blackboard.floor_height_pulses = 10000;  // 假设每层10000脉冲
    g_blackboard.motor_max_speed = 2000;       // 最大速度
    g_blackboard.motor_cruise_speed = 1500;    // 巡航速度
    g_blackboard.motor_accel_rate = 100;       // 加速度
    g_blackboard.door_timeout = 3000;          // 门超时3秒
    
    // 初始化楼层位置
    for (int i = 0; i < MAX_FLOORS; i++) {
        g_blackboard.floors[i].position_pulse = i * g_blackboard.floor_height_pulses;
    }
    
    // 初始化pending_calls数组（0不使用）
    for (int i = 0; i < 4; i++) {
        g_blackboard.pending_calls[i] = 0;
    }
    
    // 初始状态
    g_blackboard.state = ELEVATOR_CALIBRATING;
    g_blackboard.current_floor = 0;
    g_blackboard.target_floor = 0;
    g_blackboard.dir = DIR_IDLE;
    g_blackboard.door.is_closed = true;
    g_blackboard.motor_safety_ok = true;
    g_blackboard.door_safety_ok = true;
    g_blackboard.motor_running = false;
    g_blackboard.motor_speed = 100;  // 默认100%速度
    g_blackboard.digital_twin_mode = false;
    g_blackboard.timestamp = HAL_GetTick();
}

// 重置黑板
void Blackboard_Reset(void) {
    // 保存配置参数
    uint32_t floor_height = g_blackboard.floor_height_pulses;
    uint16_t max_speed = g_blackboard.motor_max_speed;
    uint16_t cruise_speed = g_blackboard.motor_cruise_speed;
    uint16_t accel_rate = g_blackboard.motor_accel_rate;
    uint32_t door_timeout = g_blackboard.door_timeout;
    
    // 清零
    memset(&g_blackboard, 0, sizeof(GlobalBlackboard_t));
    
    // 恢复配置参数
    g_blackboard.floor_height_pulses = floor_height;
    g_blackboard.motor_max_speed = max_speed;
    g_blackboard.motor_cruise_speed = cruise_speed;
    g_blackboard.motor_accel_rate = accel_rate;
    g_blackboard.door_timeout = door_timeout;
    
    // 重新初始化楼层位置
    for (int i = 0; i < MAX_FLOORS; i++) {
        g_blackboard.floors[i].position_pulse = i * g_blackboard.floor_height_pulses;
    }
}

// 推送事件到队列
bool Blackboard_PushEvent(EventType_t type, uint8_t floor) {
    return Blackboard_PushEventWithData(type, floor, NULL, 0);
}

// 推送带数据的事件到队列
bool Blackboard_PushEventWithData(EventType_t type, uint8_t floor, uint8_t* data, uint8_t len) {
    EventQueue_t* queue = &g_blackboard.event_queue;
    
    if (queue->count >= EVENT_QUEUE_SIZE) {
        return false;  // 队列满
    }
    
    Event_t* event = &queue->events[queue->tail];
    event->type = type;
    event->floor = floor;
    event->timestamp = HAL_GetTick();
    
    if (data && len > 0) {
        memcpy(event->data, data, (len > 4) ? 4 : len);
    }
    
    queue->tail = (queue->tail + 1) % EVENT_QUEUE_SIZE;
    queue->count++;
    
    return true;
}

// 从队列弹出事件
bool Blackboard_PopEvent(Event_t* event) {
    EventQueue_t* queue = &g_blackboard.event_queue;
    
    if (queue->count == 0) {
        return false;  // 队列空
    }
    
    *event = queue->events[queue->head];
    queue->head = (queue->head + 1) % EVENT_QUEUE_SIZE;
    queue->count--;
    
    return true;
}

// 检查是否有事件
bool Blackboard_HasEvents(void) {
    return g_blackboard.event_queue.count > 0;
}

// 清空事件队列
void Blackboard_ClearEvents(void) {
    g_blackboard.event_queue.head = 0;
    g_blackboard.event_queue.tail = 0;
    g_blackboard.event_queue.count = 0;
}

// 设置电梯状态
void Blackboard_SetState(ElevatorState_t new_state) {
    g_blackboard.prev_state = g_blackboard.state;
    g_blackboard.state = new_state;
}

// 获取电梯状态
ElevatorState_t Blackboard_GetState(void) {
    return g_blackboard.state;
}

// 设置待处理呼叫
void Blackboard_SetPendingCall(uint8_t floor, bool state) {
    if (floor >= 1 && floor <= 3) {
        g_blackboard.pending_calls[floor] = state ? 1 : 0;
        if (state) {
            Blackboard_PushEvent(EVENT_TARGET_UPDATED, floor - 1);  // 转换为0-2索引
        }
    }
}

// 清除待处理呼叫
void Blackboard_ClearPendingCall(uint8_t floor) {
    if (floor >= 1 && floor <= 3) {
        g_blackboard.pending_calls[floor] = 0;
    }
}

// 检查是否有待处理的呼叫
bool Blackboard_HasPendingCalls(void) {
    for (int i = 1; i <= 3; i++) {
        if (g_blackboard.pending_calls[i] != 0) {
            return true;
        }
    }
    return false;
}

// 检查指定方向是否有待处理呼叫
bool Blackboard_HasPendingInDirection(Direction_t dir) {
    int current = g_blackboard.current_floor + 1;  // 转换为1-3
    
    if (dir == DIR_UP) {
        for (int i = current + 1; i <= 3; i++) {
            if (g_blackboard.pending_calls[i] != 0) {
                return true;
            }
        }
    } else if (dir == DIR_DOWN) {
        for (int i = current - 1; i >= 1; i--) {
            if (g_blackboard.pending_calls[i] != 0) {
                return true;
            }
        }
    }
    return false;
}

// 获取下一个目标楼层（SCAN算法）
uint8_t Blackboard_GetNextTargetFloor(void) {
    int current = g_blackboard.current_floor + 1;  // 转换为1-3
    
    // 根据当前方向选择目标
    if (g_blackboard.dir == DIR_UP) {
        // 向上查找
        for (int i = current + 1; i <= 3; i++) {
            if (g_blackboard.pending_calls[i] != 0) {
                return i - 1;  // 转换回0-2
            }
        }
        // 如果上方没有，转向下方
        for (int i = 3; i >= 1; i--) {
            if (g_blackboard.pending_calls[i] != 0) {
                return i - 1;
            }
        }
    } else if (g_blackboard.dir == DIR_DOWN) {
        // 向下查找
        for (int i = current - 1; i >= 1; i--) {
            if (g_blackboard.pending_calls[i] != 0) {
                return i - 1;
            }
        }
        // 如果下方没有，转向上方
        for (int i = 1; i <= 3; i++) {
            if (g_blackboard.pending_calls[i] != 0) {
                return i - 1;
            }
        }
    } else {
        // 空闲状态，找最近的
        for (int distance = 1; distance <= 2; distance++) {
            if (current + distance <= 3 && g_blackboard.pending_calls[current + distance] != 0) {
                return current + distance - 1;
            }
            if (current - distance >= 1 && g_blackboard.pending_calls[current - distance] != 0) {
                return current - distance - 1;
            }
        }
    }
    
    return g_blackboard.current_floor;  // 没有目标楼层
}

// 更新电机位置
void Blackboard_UpdateMotorPosition(int32_t position) {
    g_blackboard.motor.current_position = position;
    g_blackboard.motor.last_update_time = HAL_GetTick();
    
    // 更新当前楼层
    g_blackboard.current_floor = Blackboard_PositionToFloor(position);
}

// 设置目标位置
void Blackboard_SetTargetPosition(int32_t position) {
    g_blackboard.motor.target_position = position;
}

// 位置转换为楼层
uint8_t Blackboard_PositionToFloor(int32_t position) {
    for (int i = MAX_FLOORS - 1; i >= 0; i--) {
        if (position >= (int32_t)(g_blackboard.floors[i].position_pulse - 
                                  g_blackboard.floor_height_pulses / 4)) {
            return i;
        }
    }
    return 0;
}

// 楼层转换为位置
int32_t Blackboard_FloorToPosition(uint8_t floor) {
    if (floor >= MAX_FLOORS) {
        floor = MAX_FLOORS - 1;
    }
    return g_blackboard.floors[floor].position_pulse;
}

// 检查是否可以安全移动
bool Blackboard_IsSafeToMove(void) {
    return g_blackboard.door.is_closed && 
           !g_blackboard.emergency_stop && 
           g_blackboard.motor_safety_ok &&
           g_blackboard.motor.is_calibrated;
}

// 检查是否可以安全开门
bool Blackboard_IsSafeToOpenDoor(void) {
    return !g_blackboard.motor.is_moving && 
           !g_blackboard.emergency_stop && 
           g_blackboard.door_safety_ok;
}

// 调试输出
void Blackboard_PrintStatus(void) {
    printf("Elevator Status:\n");
    printf("  State: %d, Floor: %d\n", g_blackboard.state, g_blackboard.current_floor);
    printf("  Motor Pos: %ld, Target: %ld\n", 
           g_blackboard.motor.current_position, 
           g_blackboard.motor.target_position);
    printf("  Door: %s\n", g_blackboard.door.is_open ? "Open" : "Closed");
    printf("  Events in queue: %d\n", g_blackboard.event_queue.count);
}