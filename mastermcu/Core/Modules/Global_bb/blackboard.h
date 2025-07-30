#ifndef __BLACKBOARD_H
#define __BLACKBOARD_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// 楼层定义
#define MAX_FLOORS 3
#define FLOOR_1 0
#define FLOOR_2 1
#define FLOOR_3 2

// 电梯状态枚举
typedef enum {
    ELEVATOR_IDLE = 0,          // 空闲状态
    ELEVATOR_MOVING_UP,         // 上行中
    ELEVATOR_MOVING_DOWN,       // 下行中
    ELEVATOR_DOOR_OPENING,      // 开门中
    ELEVATOR_DOOR_OPEN,         // 门已开
    ELEVATOR_DOOR_CLOSING,      // 关门中
    ELEVATOR_DOOR_CLOSED,       // 门已关
    ELEVATOR_EMERGENCY_STOP,    // 紧急停止
    ELEVATOR_CALIBRATING,       // 校准中
    ELEVATOR_FAULT              // 故障状态
} ElevatorState_t;

// 事件类型枚举
typedef enum {
    EVENT_NONE = 0,
    EVENT_TARGET_UPDATED,       // 目标楼层更新
    EVENT_START_MOVING,         // 开始移动
    EVENT_STOP_MOVING,          // 停止移动
    EVENT_FLOOR_REACHED,        // 到达楼层
    EVENT_OPEN_DOOR,            // 开门命令
    EVENT_CLOSE_DOOR,           // 关门命令
    EVENT_DOOR_OPENED,          // 门已开启
    EVENT_DOOR_CLOSED,          // 门已关闭
    EVENT_POSITION_ADJUST,      // 位置调整
    EVENT_SYNC_TIMEOUT,         // 同步超时
    EVENT_ERROR,                // 错误事件
    EVENT_CALL_UP,              // 上行呼叫
    EVENT_CALL_DOWN,            // 下行呼叫
    EVENT_FLOOR_REQUEST,        // 楼层请求（轿厢内）
    EVENT_POSITION_REACHED,     // 到达目标位置
    EVENT_SENSOR_TRIGGERED,     // 光电传感器触发
    EVENT_EMERGENCY_STOP,       // 紧急停止
    EVENT_CALIBRATION_DONE,     // 校准完成
    EVENT_MOTOR_FAULT,          // 电机故障
    EVENT_COMM_TIMEOUT          // 通信超时
} EventType_t;

// 事件结构体
typedef struct {
    EventType_t type;
    uint8_t floor;              // 相关楼层
    uint32_t timestamp;         // 时间戳
    uint8_t data[4];            // 附加数据
} Event_t;

// 事件队列
#define EVENT_QUEUE_SIZE 32
typedef struct {
    Event_t events[EVENT_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} EventQueue_t;

// 楼层信息
typedef struct {
    bool call_up;               // 上行呼叫按钮状态
    bool call_down;             // 下行呼叫按钮状态
    bool floor_request;         // 轿厢内楼层请求
    uint32_t position_pulse;    // 该楼层对应的脉冲位置
} FloorInfo_t;

// 电机信息
typedef struct {
    int32_t current_position;   // 当前位置（脉冲数）
    int32_t target_position;    // 目标位置（脉冲数）
    uint16_t current_speed;     // 当前速度
    uint16_t target_speed;      // 目标速度
    bool is_moving;             // 是否在运动
    bool is_calibrated;         // 是否已校准
    uint32_t last_update_time;  // 最后更新时间
} MotorInfo_t;

// 门控信息
typedef struct {
    bool is_open;               // 门是否开启
    bool is_closed;             // 门是否关闭
    bool is_moving;             // 门是否在运动
    uint8_t position;           // 门位置（0-100%）
    uint32_t last_action_time;  // 最后动作时间
} DoorInfo_t;

// 传感器信息
typedef struct {
    bool sensor_triggered[MAX_FLOORS];  // 各楼层传感器状态
    uint8_t current_floor;              // 当前楼层
    uint32_t last_trigger_time;         // 最后触发时间
} SensorInfo_t;

// 通信信息
typedef struct {
    bool slave_connected;       // Slave MCU连接状态
    uint32_t last_rx_time;      // 最后接收时间
    uint32_t last_tx_time;      // 最后发送时间
    uint16_t error_count;       // 错误计数
} CommInfo_t;

// 方向枚举
typedef enum {
    DIR_IDLE = 0,
    DIR_UP,
    DIR_DOWN
} Direction_t;

// 全局黑板结构体
typedef struct {
    // 系统状态
    ElevatorState_t state;
    ElevatorState_t prev_state;
    
    // 核心变量
    uint8_t current_floor;          // 当前楼层 (0-2对应1-3楼)
    uint8_t target_floor;           // 目标楼层
    Direction_t dir;                // 运行方向
    int pending_calls[4];           // 呼叫数组 (0未使用，1-3对应楼层)
    
    // 设备状态
    MotorInfo_t motor;
    DoorInfo_t door;
    SensorInfo_t sensors;
    CommInfo_t comm;
    
    // 辅助变量
    uint32_t timestamp;             // 系统时间戳
    uint32_t door_open_time;        // 开门时间（用于延迟关门）
    uint32_t door_timeout_start;    // 门超时开始时间
    int error_code;                 // 错误代码
    bool digital_twin_mode;         // 数字孪生模式
    int motor_speed;                // 电机速度百分比 (0-100)
    int door_retry_count;           // 门重试计数
    uint32_t rs485_last_sync_time;  // RS485最后同步时间
    bool rs485_sync_delay_flag;     // RS485同步延迟标志
    int position_offset;            // 位置偏移（脉冲数）
    
    // 事件队列
    EventQueue_t event_queue;
    
    // 运行统计
    uint32_t total_trips;           // 总行程数
    uint32_t total_distance;        // 总距离（脉冲数）
    uint32_t uptime;                // 运行时间
    
    // 配置参数
    uint32_t floor_height_pulses;   // 楼层高度（脉冲数）
    uint16_t motor_max_speed;       // 电机最大速度
    uint16_t motor_cruise_speed;    // 电机巡航速度
    uint16_t motor_accel_rate;      // 加速度
    uint32_t door_timeout;          // 门超时时间（毫秒）
    
    // 安全标志
    bool emergency_stop;            // 紧急停止标志
    bool door_safety_ok;            // 门安全状态
    bool motor_safety_ok;           // 电机安全状态
    bool motor_running;             // 电机运行标志
} GlobalBlackboard_t;

// 全局黑板实例声明
extern GlobalBlackboard_t g_blackboard;

// 黑板操作函数
void Blackboard_Init(void);
void Blackboard_Reset(void);

// 事件队列操作
bool Blackboard_PushEvent(EventType_t type, uint8_t floor);
bool Blackboard_PushEventWithData(EventType_t type, uint8_t floor, uint8_t* data, uint8_t len);
bool Blackboard_PopEvent(Event_t* event);
bool Blackboard_HasEvents(void);
void Blackboard_ClearEvents(void);

// 状态更新函数
void Blackboard_SetState(ElevatorState_t new_state);
ElevatorState_t Blackboard_GetState(void);

// 楼层请求管理
void Blackboard_SetPendingCall(uint8_t floor, bool state);
bool Blackboard_HasPendingCalls(void);
bool Blackboard_HasPendingInDirection(Direction_t dir);
uint8_t Blackboard_GetNextTargetFloor(void);
void Blackboard_ClearPendingCall(uint8_t floor);

// 位置管理
void Blackboard_UpdateMotorPosition(int32_t position);
void Blackboard_SetTargetPosition(int32_t position);
uint8_t Blackboard_PositionToFloor(int32_t position);
int32_t Blackboard_FloorToPosition(uint8_t floor);

// 安全检查
bool Blackboard_IsSafeToMove(void);
bool Blackboard_IsSafeToOpenDoor(void);

// 调试输出
void Blackboard_PrintStatus(void);

#endif /* __BLACKBOARD_H */