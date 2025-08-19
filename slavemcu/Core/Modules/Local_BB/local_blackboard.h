#ifndef __LOCAL_BLACKBOARD_H
#define __LOCAL_BLACKBOARD_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 常量定义 ==================== */
#define MAX_FLOORS          3
#define MAX_EVENT_QUEUE     16
#define DEBOUNCE_TIME_MS    100   // 防抖时间
#define MIN_RESEND_TIME_MS  500   // 最小重发间隔

/* ==================== 枚举定义 ==================== */

/* 本地事件类型 */
typedef enum {
    LOCAL_EVENT_NONE = 0,
    LOCAL_EVENT_CABIN_CALL,      // 内呼按键
    LOCAL_EVENT_PHOTO_SENSOR,    // 光电传感器
    LOCAL_EVENT_DIRECTION_SET,   // 方向设置（从Master）
    LOCAL_EVENT_DOOR_CMD,        // 门控制命令
} LocalEventType_t;

/* 门状态 */
typedef enum {
    LOCAL_DOOR_CLOSED = 0,
    LOCAL_DOOR_OPENING,
    LOCAL_DOOR_OPEN,
    LOCAL_DOOR_CLOSING
} LocalDoorState_t;

/* ==================== 结构体定义 ==================== */

/* 事件结构 */
typedef struct {
    LocalEventType_t type;
    uint8_t data1;          // 楼层号或方向
    uint8_t data2;          // 当前楼层（用于方向设置）
    uint8_t data3;          // 目标楼层（用于方向设置）
    uint32_t timestamp;
} LocalEvent_t;

/* 本地Blackboard结构 - 中央状态管理 */
typedef struct {
    /* ===== 事件队列 ===== */
    LocalEvent_t event_queue[MAX_EVENT_QUEUE];
    uint8_t event_head;
    uint8_t event_tail;
    uint8_t event_count;
    
    /* ===== 电梯状态 ===== */
    uint8_t current_floor;       // 当前楼层
    uint8_t target_floor;        // 目标楼层
    uint8_t direction;           // 0=停止, 1=上行, 2=下行
    uint8_t expected_next_floor; // 预期下一个触发楼层
    
    /* ===== 门状态 ===== */
    LocalDoorState_t door_state;
    uint32_t door_cmd_time;
    
    /* ===== 防重复发送 ===== */
    uint8_t last_sent_cabin_call;    // 最后发送的内呼楼层
    uint32_t last_cabin_call_time;   // 最后内呼发送时间
    uint8_t last_sent_photo_floor;   // 最后发送的光电楼层
    uint32_t last_photo_send_time;   // 最后光电发送时间
    
    /* ===== 统计信息 ===== */
    uint32_t cabin_call_count;       // 内呼次数
    uint32_t photo_trigger_count;    // 光电触发次数
    uint32_t rs485_send_count;       // RS485发送次数
    
    /* ===== 调试信息 ===== */
    char debug_msg[32];
    
} LocalBlackboard_t;

/* 全局实例 */
extern LocalBlackboard_t g_local_bb;

/* ==================== 函数声明 ==================== */

/* 初始化和重置 */
void LocalBB_Init(void);
void LocalBB_Reset(void);

/* 事件输入接口 - 由硬件模块调用 */
void LocalBB_AddCabinCall(uint8_t floor);
void LocalBB_AddPhotoSensor(void);  // 使用expected_next_floor
void LocalBB_SetDirection(uint8_t dir, uint8_t current, uint8_t target);
void LocalBB_AddDoorCommand(bool open);

/* 事件处理 - 主循环调用 */
void LocalBB_Process(void);
bool LocalBB_HasEvents(void);

/* 状态查询 */
uint8_t LocalBB_GetCurrentFloor(void);
uint8_t LocalBB_GetExpectedFloor(void);
uint8_t LocalBB_GetDirection(void);

/* 调试输出 */
void LocalBB_PrintStatus(void);

/* 门控命令接口 */
typedef enum {
    DOOR_CMD_NONE = 0,
    DOOR_CMD_OPEN,
    DOOR_CMD_CLOSE
} LocalBB_DoorCommand_t;

LocalBB_DoorCommand_t LocalBB_GetDoorCommand(void);
void LocalBB_ClearDoorCommand(void);

#endif /* __LOCAL_BLACKBOARD_H */