#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* 步进电机常量 */
#define STEPPER_NODE_ID         32      // 电机节点ID
#define STEPPER_SPEED           10000   // 运行速度 (p/s)

/* CANopen对象字典索引 */
#define OD_CONTROL_WORD         0x6040
#define OD_STATUS_WORD          0x6041
#define OD_MODE_OF_OPERATION    0x6060
#define OD_TARGET_POSITION      0x607A
#define OD_ACTUAL_POSITION      0x6064
#define OD_PROFILE_VELOCITY     0x6081
#define OD_PROFILE_ACCELERATION 0x6083
#define OD_PROFILE_DECELERATION 0x6084
#define OD_PEAK_CURRENT         0x2000  // 驱动器峰值电流
#define OD_SUBDIVISION          0x2001  // 驱动器细分数
#define OD_MIN_CURRENT          0x2003  // 最小电流参数
#define OD_MOTOR_ENABLE_SELECT  0x3200  // 电机使能选择

/* 控制字定义 - 根据立三电机文档 */
#define CW_SHUTDOWN             0x0006  // 脱机
#define CW_ENABLE               0x0007  // 使能锁机
#define CW_ENABLE_OPERATION     0x000F  // 标准使能运行
#define CW_START_MOVE           0x001F  // 开始移动
#define CW_ABS_REL_BIT          0x0040  // bit6: 0=绝对位置, 1=相对位置
#define CW_NEW_SETPOINT         0x0010  // bit4: 新设定点
#define CW_CHANGE_IMMEDIATELY   0x0020  // bit5: 立即改变
#define CW_HALT                 0x010F  // 停止

/* 状态字定义 */
#define SW_READY_TO_SWITCH_ON   0x0001
#define SW_SWITCHED_ON          0x0002
#define SW_OPERATION_ENABLED    0x0004
#define SW_TARGET_REACHED       0x0400
#define SW_SETPOINT_ACK         0x1000

/* 操作模式 */
#define MODE_PROFILE_POSITION   1

/* 步进电机结构体 - 仅包含底层驱动相关 */
typedef struct {
    /* 基本参数 */
    uint8_t node_id;                    // 节点ID
    
    /* CANopen状态 */
    uint16_t control_word;              // 控制字
    uint16_t status_word;               // 状态字
    
    /* 位置信息 */
    int32_t current_position;           // 当前位置（步数）
    int32_t target_position;            // 目标位置（步数）
    
    /* 通信状态 */
    uint32_t last_response_time;        // 最后响应时间
    bool is_connected;                  // 连接状态
    bool is_enabled;                    // 使能状态
    bool is_moving;                     // 运动状态
    
} StepperMotor_t;

/* 初始化和基本控制 */
void StepperMotor_Init(StepperMotor_t* motor);
void StepperMotor_Enable(StepperMotor_t* motor);
void StepperMotor_Disable(StepperMotor_t* motor);
void StepperMotor_Reset(StepperMotor_t* motor);

/* 位置控制 - 底层接口 */
void StepperMotor_MoveAbsolute(StepperMotor_t* motor, int32_t position);
void StepperMotor_MoveRelative(StepperMotor_t* motor, int32_t steps);
void StepperMotor_Stop(StepperMotor_t* motor);

/* 状态查询 */
bool StepperMotor_IsMoving(StepperMotor_t* motor);
bool StepperMotor_IsEnabled(StepperMotor_t* motor);
bool StepperMotor_IsConnected(StepperMotor_t* motor);
int32_t StepperMotor_GetPosition(StepperMotor_t* motor);
uint16_t StepperMotor_GetStatus(StepperMotor_t* motor);

/* 周期性更新 - 必须在主循环调用 */
void StepperMotor_Update(StepperMotor_t* motor);

/* CAN通信接口 */
bool StepperMotor_SendSDO(uint8_t node_id, uint16_t index, uint8_t subindex, 
                          uint8_t* data, uint8_t len);
bool StepperMotor_ReadSDO(uint8_t node_id, uint16_t index, uint8_t subindex, 
                          uint8_t* data, uint8_t* len);

#endif /* __STEPPER_MOTOR_H */