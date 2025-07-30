#ifndef __SERVO_CONTROL_H
#define __SERVO_CONTROL_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "local_blackboard.h"

// 舵机参数
#define SERVO_MIN_ANGLE         0       // 最小角度（关门）
#define SERVO_MAX_ANGLE         180     // 最大角度（开门）
#define SERVO_SPEED             50      // 舵机速度（度/秒）
#define SERVO_TIMEOUT           3000    // 超时时间（ms）
#define SERVO_POSITION_TOLERANCE 5      // 位置容差（度）
#define SERVO_RETRY_MAX         3       // 最大重试次数

// 舵机命令
#define SERVO_CMD_SET_POSITION  0x01    // 设置位置
#define SERVO_CMD_GET_POSITION  0x02    // 获取位置
#define SERVO_CMD_SET_SPEED     0x03    // 设置速度
#define SERVO_CMD_GET_STATUS    0x04    // 获取状态
#define SERVO_CMD_STOP          0x05    // 停止

// 舵机状态枚举
typedef enum {
    SERVO_STATE_IDLE = 0,           // 空闲
    SERVO_STATE_SENDING_CMD,        // 发送命令中
    SERVO_STATE_WAIT_FEEDBACK,      // 等待反馈
    SERVO_STATE_ADJUST,             // 调整位置
    SERVO_STATE_TIMEOUT,            // 超时
    SERVO_STATE_ERROR              // 错误
} ServoState_t;

// 舵机控制结构体
typedef struct {
    // FSM状态
    ServoState_t current_state;
    ServoState_t prev_state;
    
    // 私有变量
    uint32_t start_time;            // 动作开始时间
    uint32_t state_entry_time;      // 状态进入时间
    int retry_count;                // 重试计数
    int target_pos;                 // 目标位置（度）
    int current_pos;                // 当前位置（度）
    int last_pos;                   // 上次位置
    
    // 通信相关
    uint8_t tx_buffer[16];          // 发送缓冲区
    uint8_t rx_buffer[16];          // 接收缓冲区
    uint8_t rx_index;               // 接收索引
    bool waiting_response;          // 等待响应标志
    uint8_t last_cmd;               // 最后发送的命令
    
    // 门控制相关
    bool door_opening;              // 开门中
    bool door_closing;              // 关门中
    uint32_t door_action_start;     // 门动作开始时间
    
    // 错误管理
    uint8_t error_count;            // 错误计数
    uint16_t error_code;            // 错误代码
    
    // 反馈相关
    bool has_feedback;              // 有反馈标志
    int feedback_position;          // 反馈位置
    uint8_t feedback_status;        // 反馈状态
    
} ServoControl_t;

// 舵机反馈数据结构
typedef struct {
    uint8_t status;                 // 状态字节
    int16_t position;               // 位置（度）
    int16_t speed;                  // 速度（度/秒）
    uint8_t error_flags;            // 错误标志
} ServoFeedback_t;

// 全局舵机控制实例
extern ServoControl_t g_servo_control;

// 初始化和更新函数
void ServoControl_Init(void);
void ServoControl_Update(void);

// 门控制函数
void ServoControl_OpenDoor(void);
void ServoControl_CloseDoor(void);
void ServoControl_StopDoor(void);
bool ServoControl_IsDoorOpen(void);
bool ServoControl_IsDoorClosed(void);
bool ServoControl_IsDoorMoving(void);

// 位置控制函数
void ServoControl_SetPosition(int position);
int ServoControl_GetPosition(void);
bool ServoControl_IsAtPosition(int position);

// 内部处理函数
bool ServoControl_SendCommand(uint8_t cmd, uint8_t* params, uint8_t param_len);
bool ServoControl_ReadFeedback(ServoFeedback_t* feedback);
int ServoControl_CalculateAdjust(int error);
void ServoControl_ProcessFeedback(void);

// UART通信函数
void ServoControl_UART_Init(void);
void ServoControl_UART_SendByte(uint8_t data);
void ServoControl_UART_RxCallback(uint8_t data);
bool ServoControl_WaitResponse(uint32_t timeout);

// 状态查询
ServoState_t ServoControl_GetState(void);
bool ServoControl_HasError(void);
uint16_t ServoControl_GetErrorCode(void);

// 调试函数
void ServoControl_PrintStatus(void);

// 状态机处理函数（内部）
void ServoControl_HandleIdle(void);
void ServoControl_HandleSendingCmd(void);
void ServoControl_HandleWaitFeedback(void);
void ServoControl_HandleAdjust(void);
void ServoControl_HandleTimeout(void);
void ServoControl_HandleError(void);

#endif /* __SERVO_CONTROL_H */