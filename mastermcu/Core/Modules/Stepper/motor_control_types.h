#ifndef MOTOR_CONTROL_TYPES_H
#define MOTOR_CONTROL_TYPES_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// 仅保留必要的类型定义，实际功能由CANopen提供
// 此文件仅用于向后兼容

// 电机状态定义
#define MOTOR_STATUS_IDLE     0x00
#define MOTOR_STATUS_RUNNING  0x01
#define MOTOR_STATUS_FAULT    0x02
#define MOTOR_STATUS_TIMEOUT  0x03

// 电机结构体
typedef struct {
    uint16_t motor_can_id;          // 电机CAN ID
    int32_t current_position;       // 当前位置
    int32_t target_position;        // 目标位置
    bool is_moving;                 // 运动状态
    uint16_t speed;                 // 当前速度
    uint8_t status;                 // 电机状态
    uint32_t last_command_time;     // 上次发送命令时间
    uint32_t last_response_time;    // 上次收到响应时间
} Motor_t;

// 函数声明（实际实现在adapter中）
void Motor_Init(Motor_t* motor, uint16_t motor_id);
uint8_t Motor_MoveTo(Motor_t* motor, int32_t position);
uint8_t Motor_MoveSteps(Motor_t* motor, int32_t steps);
uint8_t Motor_SetSpeed(Motor_t* motor, uint16_t speed);
uint8_t Motor_Stop(Motor_t* motor);
uint8_t Motor_SetZero(Motor_t* motor);
uint8_t Motor_ReadPosition(Motor_t* motor);
uint8_t Motor_ReadStatus(Motor_t* motor);
void Motor_ProcessRxMessage(Motor_t* motor, uint16_t rx_id, uint8_t* data);
void Motor_Task(Motor_t* motor);

#endif // MOTOR_CONTROL_TYPES_H