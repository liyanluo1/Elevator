#ifndef ELEVATOR_TYPES_H
#define ELEVATOR_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 基本常量定义 ==================== */

// 楼层定义
#define MAX_FLOORS      3
#define FLOOR_1         0
#define FLOOR_2         1
#define FLOOR_3         2

// 无效值定义
#define INVALID_FLOOR   0xFF

/* ==================== 枚举类型定义 ==================== */

// 运动方向
typedef enum {
    DIR_IDLE = 0,   // 静止
    DIR_UP   = 1,   // 上行
    DIR_DOWN = 2    // 下行
} Direction_t;

// 门状态
typedef enum {
    DOOR_CLOSED  = 0,   // 门关闭
    DOOR_OPENING = 1,   // 正在开门
    DOOR_OPEN    = 2,   // 门打开
    DOOR_CLOSING = 3    // 正在关门
} DoorState_t;

// 电梯运行状态
typedef enum {
    ELEVATOR_IDLE        = 0,   // 空闲
    ELEVATOR_MOVING      = 1,   // 运行中
    ELEVATOR_DOOR_OPEN   = 2,   // 开门状态
    ELEVATOR_CALIBRATING = 3,   // 校准中
    ELEVATOR_ERROR       = 4    // 错误状态
} ElevatorState_t;

// 按钮类型
typedef enum {
    BUTTON_NONE     = 0,
    BUTTON_CALL_UP  = 1,    // 外呼上行
    BUTTON_CALL_DOWN = 2,   // 外呼下行
    BUTTON_FLOOR    = 3     // 内选楼层
} ButtonType_t;

// 错误代码
typedef enum {
    ERROR_NONE          = 0x00,
    ERROR_MOTOR_FAULT   = 0x01,
    ERROR_DOOR_FAULT    = 0x02,
    ERROR_SENSOR_FAULT  = 0x04,
    ERROR_COMM_TIMEOUT  = 0x08,
    ERROR_CALIBRATION   = 0x10,
    ERROR_OVERLOAD      = 0x20,
    ERROR_EMERGENCY     = 0x40
} ErrorCode_t;

/* ==================== 事件系统定义 ==================== */

// 事件类型
typedef enum {
    EVENT_NONE              = 0,
    EVENT_BUTTON_PRESSED    = 1,
    EVENT_FLOOR_REACHED     = 2,
    EVENT_DOOR_OPENED       = 3,
    EVENT_DOOR_CLOSED       = 4,
    EVENT_CALIBRATION_DONE  = 5,
    EVENT_ERROR_OCCURRED    = 6,
    EVENT_EMERGENCY_STOP    = 7,
    EVENT_RS485_RECEIVED    = 8
} EventType_t;

// 事件结构体
typedef struct {
    EventType_t type;       // 事件类型
    uint8_t floor;         // 相关楼层
    uint8_t data;          // 附加数据
    uint32_t timestamp;    // 时间戳
} Event_t;

// 事件队列大小
#define EVENT_QUEUE_SIZE    16

/* ==================== 按钮请求定义 ==================== */

// 按钮请求结构
typedef struct {
    bool call_up[MAX_FLOORS];      // 外呼上行请求
    bool call_down[MAX_FLOORS];    // 外呼下行请求  
    bool floor_request[MAX_FLOORS]; // 内选楼层请求
} ButtonRequests_t;

/* ==================== 通信协议定义 ==================== */

// RS485命令类型
typedef enum {
    CMD_STATUS_UPDATE   = 0x01,
    CMD_BUTTON_EVENT    = 0x02,
    CMD_SENSOR_EVENT    = 0x03,
    CMD_DOOR_CONTROL    = 0x04,
    CMD_ERROR_REPORT    = 0x05,
    CMD_HEARTBEAT       = 0x06,
    CMD_CONFIG_SET      = 0x07,
    CMD_CONFIG_GET      = 0x08
} CommandType_t;

// 通信状态
typedef enum {
    COMM_OK         = 0,
    COMM_TIMEOUT    = 1,
    COMM_ERROR      = 2,
    COMM_BUSY       = 3
} CommStatus_t;

/* ==================== 时间相关定义 ==================== */

// 超时时间定义（毫秒）
#define TIMEOUT_DOOR_OPERATION  5000    // 开关门超时
#define TIMEOUT_FLOOR_TRAVEL    30000   // 楼层间移动超时
#define TIMEOUT_CALIBRATION     60000   // 校准超时
#define TIMEOUT_RS485_RESPONSE  100     // RS485响应超时
#define TIMEOUT_HEARTBEAT       1000    // 心跳超时

// 延时时间定义（毫秒）
#define DELAY_DOOR_WAIT         3000    // 门保持打开时间
#define DELAY_BUTTON_DEBOUNCE   50      // 按钮去抖时间
#define DELAY_SENSOR_DEBOUNCE   20      // 传感器去抖时间

/* ==================== 工具宏定义 ==================== */

// 范围检查
#define IS_VALID_FLOOR(floor)   ((floor) < MAX_FLOORS)
#define IS_VALID_DIRECTION(dir) ((dir) >= DIR_IDLE && (dir) <= DIR_DOWN)

// 位操作 - 避免与STM32 HAL宏冲突
#ifndef SET_BIT
#define SET_BIT(value, bit)     ((value) |= (1U << (bit)))
#endif
#ifndef CLEAR_BIT
#define CLEAR_BIT(value, bit)   ((value) &= ~(1U << (bit)))
#endif
#define CHECK_BIT(value, bit)   (((value) >> (bit)) & 1U)
#define TOGGLE_BIT(value, bit)  ((value) ^= (1U << (bit)))

// 最大最小值
#ifndef MAX
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
#endif

// 数组大小
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

#endif /* ELEVATOR_TYPES_H */