#ifndef __LOCAL_BLACKBOARD_H
#define __LOCAL_BLACKBOARD_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// 楼层定义 (与master保持一致)
#define MAX_FLOORS 3
#define FLOOR_1 0
#define FLOOR_2 1
#define FLOOR_3 2

// 电梯状态枚举 (子集)
typedef enum {
    ELEVATOR_IDLE = 0,
    ELEVATOR_DOOR_OPENING,
    ELEVATOR_DOOR_OPEN,
    ELEVATOR_DOOR_CLOSING,
    ELEVATOR_DOOR_CLOSED,
    ELEVATOR_DOOR_ERROR,
    ELEVATOR_SENSOR_CALIBRATING
} LocalElevatorState_t;

// 门状态枚举
typedef enum {
    DOOR_CLOSED = 0,
    DOOR_OPENING,
    DOOR_OPEN,
    DOOR_CLOSING,
    DOOR_ERROR
} DoorState_t;

// 事件类型枚举 (子集)
typedef enum {
    EVENT_NONE = 0,
    EVENT_TARGET_UPDATED,       // 键盘输入新目标
    EVENT_FLOOR_REACHED,        // 光电传感器检测到楼层
    EVENT_OPEN_DOOR,            // 开门命令
    EVENT_CLOSE_DOOR,           // 关门命令
    EVENT_DOOR_OPENED,          // 门已开启
    EVENT_DOOR_CLOSED,          // 门已关闭
    EVENT_POSITION_ADJUST,      // 位置校准调整
    EVENT_SYNC_TIMEOUT,         // RS485同步超时
    EVENT_ERROR,                // 错误事件
    EVENT_SENSOR_TRIGGERED,     // 光电传感器触发
    EVENT_KEYBOARD_INPUT        // 键盘输入事件
} LocalEventType_t;

// 事件结构体
typedef struct {
    LocalEventType_t type;
    int data;                   // 通用数据（楼层号、偏移量等）
    uint32_t timestamp;         // 时间戳
} LocalEvent_t;

// 事件队列 (较小)
#define LOCAL_EVENT_QUEUE_SIZE 10
typedef struct {
    LocalEvent_t events[LOCAL_EVENT_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} LocalEventQueue_t;

// 方向枚举
typedef enum {
    DIR_IDLE = 0,
    DIR_UP,
    DIR_DOWN
} Direction_t;

// 本地黑板结构体
typedef struct {
    // 核心变量
    uint8_t target_floor;           // 目标楼层（键盘输入）
    DoorState_t door;               // 门状态
    uint8_t current_floor;          // 当前楼层（光电传感器）
    bool digital_twin_mode;         // 数字孪生模式
    LocalElevatorState_t state;     // 本地状态
    
    // 辅助变量
    uint32_t timestamp;             // 系统时间戳
    uint32_t door_timeout_start;    // 门超时开始时间
    int error_code;                 // 错误代码
    int door_retry_count;           // 门重试计数
    uint32_t rs485_last_sync_time;  // RS485最后同步时间
    int position_offset;            // 位置偏移（光电传感器校准）
    int trigger_count;              // 光电传感器触发计数
    
    // 舵机相关
    int servo_position;             // 舵机当前位置（0-180度）
    int servo_target_position;      // 舵机目标位置
    bool servo_busy;                // 舵机忙标志
    
    // 光电传感器相关
    bool sensor_triggered;          // 光电传感器触发状态
    uint32_t sensor_last_trigger;   // 最后触发时间
    Direction_t last_direction;     // 最后运动方向（用于判断楼层）
    
    // 键盘相关
    uint8_t keyboard_buffer;        // 键盘输入缓冲
    bool keyboard_new_input;        // 新输入标志
    
    // 事件队列
    LocalEventQueue_t event_queue;
    
    // 同步标志
    bool need_sync;                 // 需要同步到master
    uint32_t sync_fields;           // 需要同步的字段位掩码
} LocalBlackboard_t;

// 同步字段掩码定义
#define SYNC_FIELD_TARGET_FLOOR     (1 << 0)
#define SYNC_FIELD_DOOR             (1 << 1)
#define SYNC_FIELD_CURRENT_FLOOR    (1 << 2)
#define SYNC_FIELD_ERROR            (1 << 3)
#define SYNC_FIELD_POSITION_OFFSET  (1 << 4)

// 全局本地黑板实例声明
extern LocalBlackboard_t g_local_bb;

// 黑板操作函数
void LocalBlackboard_Init(void);
void LocalBlackboard_Reset(void);

// 事件队列操作
bool LocalBlackboard_PushEvent(LocalEventType_t type, int data);
bool LocalBlackboard_PopEvent(LocalEvent_t* event);
bool LocalBlackboard_HasEvents(void);
void LocalBlackboard_ClearEvents(void);

// 状态更新函数
void LocalBlackboard_SetState(LocalElevatorState_t new_state);
LocalElevatorState_t LocalBlackboard_GetState(void);

// 门控制函数
void LocalBlackboard_SetDoorState(DoorState_t state);
DoorState_t LocalBlackboard_GetDoorState(void);

// 楼层管理
void LocalBlackboard_SetCurrentFloor(uint8_t floor);
void LocalBlackboard_SetTargetFloor(uint8_t floor);

// 同步管理
void LocalBlackboard_MarkForSync(uint32_t fields);
bool LocalBlackboard_NeedsSync(void);
uint32_t LocalBlackboard_GetSyncFields(void);
void LocalBlackboard_ClearSyncFlags(void);

// 光电传感器相关
void LocalBlackboard_UpdateSensorState(bool triggered);
uint8_t LocalBlackboard_CalculateFloor(Direction_t dir, int trigger_count);

// 键盘相关
void LocalBlackboard_SetKeyboardInput(uint8_t key);
bool LocalBlackboard_HasNewKeyboardInput(void);
uint8_t LocalBlackboard_GetKeyboardInput(void);

// 调试输出
void LocalBlackboard_PrintStatus(void);

#endif /* __LOCAL_BLACKBOARD_H */