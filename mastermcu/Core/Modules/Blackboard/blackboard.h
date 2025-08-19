#ifndef __BLACKBOARD_H
#define __BLACKBOARD_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 系统常量 ==================== */
#define MAX_FLOORS          3       // 最大楼层数
#define STEPS_PER_FLOOR     10800   // 每层步数

/* 时间配置（毫秒） */
#define DOOR_OPENING_TIME_MS   2500  // 开门时间
#define DOOR_OPEN_HOLD_TIME_MS 3000  // 保持开门时间
#define DOOR_CLOSING_TIME_MS   2500  // 关门时间

/* ==================== 枚举定义 ==================== */

/* 电机控制命令 */
typedef enum {
    MOTOR_CMD_NONE = 0,
    MOTOR_CMD_MOVE_TO,         // 移动到指定位置
    MOTOR_CMD_MOVE_TO_FLOOR,   // 移动到指定楼层
    MOTOR_CMD_STOP,            // 停止
    MOTOR_CMD_EMERGENCY_STOP,  // 紧急停止
    MOTOR_CMD_ENABLE,          // 使能
    MOTOR_CMD_DISABLE          // 失能
} MotorCommand_t;

/* 电梯状态 */
typedef enum {
    STATE_IDLE,           // 空闲等待
    STATE_MOVING,         // 移动中
    STATE_DOOR_OPERATING, // 门操作中（开门+保持+关门）
    STATE_PREPARING       // 准备下次移动
} ElevatorState_t;

/* 运动方向 */
typedef enum {
    DIR_IDLE = 0,    // 静止
    DIR_UP = 1,      // 上行
    DIR_DOWN = 2     // 下行
} Direction_t;

/* 门状态（简化版，Master端） */
typedef enum {
    DOOR_CLOSED = 0,
    DOOR_OPENING,
    DOOR_OPEN,
    DOOR_CLOSING
} DoorState_t;

/* 事件类型 */
typedef enum {
    EVENT_NONE = 0,
    EVENT_BUTTON_UP,        // 外呼上行按钮
    EVENT_BUTTON_DOWN,      // 外呼下行按钮
    EVENT_BUTTON_PRESS,     // 通用按钮按下
    EVENT_CABIN_CALL,       // 内呼（轿厢按钮）
    EVENT_PHOTO_SENSOR,     // 光电传感器触发
    EVENT_ARRIVED,          // 到达目标位置
    EVENT_TIMER_EXPIRED,    // 定时器到期
    EVENT_TIMEOUT,          // 超时事件
    EVENT_DOOR_CMD_DONE     // 门命令完成
} EventType_t;

/* 事件结构 */
typedef struct {
    EventType_t type;
    uint8_t floor;          // 相关楼层 (1-3)
    uint32_t timestamp;
} Event_t;

/* ==================== Blackboard结构 ==================== */
typedef struct {
    /* 状态管理 */
    ElevatorState_t state;
    ElevatorState_t prev_state;
    uint32_t state_enter_time;
    
    /* 位置信息 */
    uint8_t current_floor;      // 当前楼层 (1-3)
    uint8_t target_floor;       // 目标楼层 (1-3)
    uint8_t next_target_floor;  // 下一个目标楼层（中途停靠后继续）
    Direction_t direction;      // 运行方向
    int32_t motor_position;     // 电机当前位置
    int32_t encoder_offset;     // 编码器偏移
    
    /* 呼叫请求 (索引0不用，1-3对应楼层) */
    bool up_calls[4];           // 上行外呼
    bool down_calls[4];         // 下行外呼  
    bool cabin_calls[4];        // 内呼
    
    /* 门控制 */
    DoorState_t door_state;
    uint32_t door_timer;        // 门操作计时器
    
    /* 事件队列 */
    Event_t events[16];
    uint8_t event_head;
    uint8_t event_tail;
    uint8_t event_count;
    
    /* 电机控制命令 */
    MotorCommand_t motor_command;      // 当前电机命令
    int32_t motor_command_param;       // 命令参数
    
    /* 调试信息 */
    char debug_msg[32];
    uint32_t last_photo_floor;  // 最后光电检测楼层
    
    /* 统计信息 */
    uint32_t total_trips;
    uint32_t total_stops;
    
} Blackboard_t;

/* ==================== 全局实例 ==================== */
extern Blackboard_t g_blackboard;

/* ==================== 函数声明 ==================== */

/* 初始化 */
void Blackboard_Init(void);
void Blackboard_Reset(void);

/* 状态管理 */
void Blackboard_SetState(ElevatorState_t new_state);
const char* Blackboard_GetStateName(ElevatorState_t state);

/* 事件管理 */
bool Blackboard_PushEvent(EventType_t type, uint8_t floor);
bool Blackboard_PopEvent(Event_t* event);
bool Blackboard_HasEvents(void);
void Blackboard_ClearEvents(void);

/* 请求管理 */
void Blackboard_AddUpCall(uint8_t floor);
void Blackboard_AddDownCall(uint8_t floor);
void Blackboard_AddCabinCall(uint8_t floor);
void Blackboard_ClearCall(uint8_t floor);
bool Blackboard_HasCallAt(uint8_t floor);
bool Blackboard_HasAnyCall(void);

/* SCAN调度算法 */
uint8_t Blackboard_GetNextTarget(void);
bool Blackboard_HasCallsInDirection(Direction_t dir);
void Blackboard_UpdateDirection(void);

/* 位置管理 */
void Blackboard_UpdateMotorPosition(int32_t position);
int32_t Blackboard_GetTargetPosition(uint8_t floor);
bool Blackboard_IsNearTarget(void);
void Blackboard_CalibratePosition(uint8_t floor);
void Blackboard_SetEncoderOffset(int32_t offset);

/* 电机控制接口 - 解耦FSM和Stepper */
void Blackboard_SetMotorCommand(MotorCommand_t cmd, int32_t param);
MotorCommand_t Blackboard_GetMotorCommand(void);
int32_t Blackboard_GetMotorCommandParam(void);
void Blackboard_ClearMotorCommand(void);

/* 调试 */
void Blackboard_UpdateDebugMsg(const char* msg);
void Blackboard_PrintStatus(void);

#endif /* __BLACKBOARD_H */