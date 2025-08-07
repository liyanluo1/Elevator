#ifndef __CANOPEN_STEPPER_H
#define __CANOPEN_STEPPER_H

#include "stm32f4xx_hal.h"
#include "../../CAN/can.h"
#include <stdbool.h>
#include <stdint.h>

// CANopen DS402 对象字典索引定义
// 设备控制
#define OD_CONTROL_WORD         0x6040  // 控制字
#define OD_STATUS_WORD          0x6041  // 状态字
#define OD_OPERATION_MODE       0x6060  // 运行模式
#define OD_OPERATION_MODE_DISP  0x6061  // 运行模式显示

// 位置控制
#define OD_TARGET_POSITION      0x607A  // 目标位置
#define OD_POSITION_ACTUAL      0x6064  // 实际位置
#define OD_VELOCITY_ACTUAL      0x606C  // 实际速度

// 轮廓参数
#define OD_PROFILE_VELOCITY     0x6081  // 轮廓速度
#define OD_PROFILE_ACCELERATION 0x6083  // 轮廓加速度
#define OD_PROFILE_DECELERATION 0x6084  // 轮廓减速度

// 速度模式
#define OD_TARGET_VELOCITY      0x60FF  // 目标速度

// 回原点模式
#define OD_HOMING_METHOD        0x6098  // 回原方式
#define OD_HOMING_SPEEDS        0x6099  // 回原速度
#define OD_HOMING_ACCELERATION  0x609A  // 回原加速度

// 力矩模式
#define OD_TARGET_TORQUE        0x6071  // 目标力矩
#define OD_MAX_TORQUE           0x6072  // 最大力矩

// 电机参数
#define OD_MOTOR_RATED_CURRENT  0x2000  // 额定电流
#define OD_MOTOR_SUBDIVISION    0x2001  // 细分设置
#define OD_TORQUE_SUBMODE       0x2158  // 力矩子模式 (子索引01: 恒力矩=3)

// 运行模式定义
typedef enum {
    MODE_NO_MODE = 0,
    MODE_PROFILE_POSITION = 1,    // PP - 轮廓位置模式
    MODE_VELOCITY = 2,            // VL - 速度模式（未使用）
    MODE_PROFILE_VELOCITY = 3,    // PV - 轮廓速度模式
    MODE_PROFILE_TORQUE = 4,      // PT - 轮廓力矩模式
    MODE_HOMING = 6,              // HM - 回原点模式
    MODE_INTERPOLATED_POSITION = 7 // IP - 插补位置模式
} OperationMode_t;

// DS402状态机状态
typedef enum {
    STATE_NOT_READY = 0,
    STATE_SWITCH_ON_DISABLED,
    STATE_READY_TO_SWITCH_ON,
    STATE_SWITCHED_ON,
    STATE_OPERATION_ENABLED,
    STATE_QUICK_STOP_ACTIVE,
    STATE_FAULT_REACTION_ACTIVE,
    STATE_FAULT
} DS402State_t;

// 控制字位定义
typedef union {
    uint16_t value;
    struct {
        uint16_t switch_on : 1;           // bit0
        uint16_t enable_voltage : 1;      // bit1
        uint16_t quick_stop : 1;          // bit2 (0有效)
        uint16_t enable_operation : 1;    // bit3
        uint16_t new_setpoint : 1;        // bit4 (PP模式)
        uint16_t change_set_immediately : 1; // bit5 (PP模式)
        uint16_t abs_rel : 1;             // bit6 (PP模式: 0=绝对, 1=相对)
        uint16_t fault_reset : 1;         // bit7
        uint16_t halt : 1;                // bit8
        uint16_t reserved : 7;            // bit9-15
    } bits;
} ControlWord_t;

// 状态字位定义
typedef union {
    uint16_t value;
    struct {
        uint16_t ready_to_switch_on : 1;  // bit0
        uint16_t switched_on : 1;          // bit1
        uint16_t operation_enabled : 1;    // bit2
        uint16_t fault : 1;                // bit3
        uint16_t voltage_enabled : 1;      // bit4
        uint16_t quick_stop : 1;           // bit5 (0有效)
        uint16_t switch_on_disabled : 1;   // bit6
        uint16_t warning : 1;              // bit7
        uint16_t manufacturer_specific : 1;// bit8
        uint16_t remote : 1;               // bit9
        uint16_t target_reached : 1;       // bit10
        uint16_t internal_limit_active : 1;// bit11
        uint16_t op_mode_specific1 : 1;    // bit12
        uint16_t op_mode_specific2 : 1;    // bit13
        uint16_t reserved : 2;             // bit14-15
    } bits;
} StatusWord_t;

// SDO传输结构
typedef struct {
    uint8_t cs;        // 命令说明符
    uint16_t index;    // 对象字典索引
    uint8_t subindex;  // 子索引
    uint8_t data[4];   // 数据（最多4字节）
} SDO_t;

// 步进电机控制结构
typedef struct {
    // CAN通信
    uint8_t node_id;              // CANopen节点ID
    uint16_t rx_cob_id;           // 接收COB-ID (0x600 + node_id)
    uint16_t tx_cob_id;           // 发送COB-ID (0x580 + node_id)
    
    // DS402状态机
    DS402State_t state;           // 当前状态
    ControlWord_t control_word;   // 控制字
    StatusWord_t status_word;     // 状态字
    
    // 运行模式
    OperationMode_t operation_mode;
    
    // 位置控制
    int32_t target_position;      // 目标位置
    int32_t actual_position;      // 实际位置
    uint32_t profile_velocity;    // 轮廓速度
    uint32_t profile_acc;         // 轮廓加速度
    uint32_t profile_dec;         // 轮廓减速度
    
    // 速度控制
    int32_t target_velocity;      // 目标速度
    int32_t actual_velocity;      // 实际速度
    
    // 回原点
    uint8_t homing_method;        // 回原方式
    uint32_t homing_speed_fast;   // 快速回原速度
    uint32_t homing_speed_slow;   // 慢速回原速度
    uint32_t homing_acceleration; // 回原加速度
    
    // 电机参数
    uint16_t motor_rated_current; // 额定电流(mA)
    uint16_t motor_subdivision;   // 细分数
    
    // 通信状态
    uint32_t last_sdo_time;       // 上次SDO通信时间
    uint32_t last_heartbeat_time; // 上次心跳时间
    bool is_connected;            // 连接状态
    
} CANopen_Stepper_t;

// 初始化函数
void CANopen_Stepper_Init(CANopen_Stepper_t* stepper, uint8_t node_id);

// NMT (Network Management) 功能
uint8_t CANopen_NMT_StartNode(uint8_t node_id);
uint8_t CANopen_NMT_PreOperational(uint8_t node_id);

// 力矩模式控制函数
uint8_t CANopen_SetTorqueMode(CANopen_Stepper_t* stepper);
uint8_t CANopen_EnableTorqueMode(CANopen_Stepper_t* stepper);
uint8_t CANopen_SetTargetTorque(CANopen_Stepper_t* stepper, int16_t torque);

// 状态机控制
uint8_t CANopen_ReadyToSwitchOn(CANopen_Stepper_t* stepper);
uint8_t CANopen_SwitchedOn(CANopen_Stepper_t* stepper);
uint8_t CANopen_OperationEnable(CANopen_Stepper_t* stepper);

// SDO通信函数
uint8_t CANopen_SDO_Write(CANopen_Stepper_t* stepper, uint16_t index, uint8_t subindex, 
                         void* data, uint8_t size);
uint8_t CANopen_SDO_Read(CANopen_Stepper_t* stepper, uint16_t index, uint8_t subindex);

// DS402状态机控制
uint8_t CANopen_SetState(CANopen_Stepper_t* stepper, DS402State_t target_state);
DS402State_t CANopen_GetState(CANopen_Stepper_t* stepper);
uint8_t CANopen_EnableOperation(CANopen_Stepper_t* stepper);
uint8_t CANopen_DisableOperation(CANopen_Stepper_t* stepper);
uint8_t CANopen_QuickStop(CANopen_Stepper_t* stepper);
uint8_t CANopen_FaultReset(CANopen_Stepper_t* stepper);

// 运行模式控制
uint8_t CANopen_SetOperationMode(CANopen_Stepper_t* stepper, OperationMode_t mode);
OperationMode_t CANopen_GetOperationMode(CANopen_Stepper_t* stepper);

// 位置模式(PP)
uint8_t CANopen_SetTargetPosition(CANopen_Stepper_t* stepper, int32_t position, bool absolute);
uint8_t CANopen_SetProfileVelocity(CANopen_Stepper_t* stepper, uint32_t velocity);
uint8_t CANopen_SetProfileAcceleration(CANopen_Stepper_t* stepper, uint32_t acc, uint32_t dec);
uint8_t CANopen_StartPositionMove(CANopen_Stepper_t* stepper);
bool CANopen_IsTargetReached(CANopen_Stepper_t* stepper);

// 速度模式(PV)
uint8_t CANopen_SetTargetVelocity(CANopen_Stepper_t* stepper, int32_t velocity);
uint8_t CANopen_StartVelocityMode(CANopen_Stepper_t* stepper);
uint8_t CANopen_HaltVelocityMode(CANopen_Stepper_t* stepper);

// 回原点模式(HM)
uint8_t CANopen_SetHomingMethod(CANopen_Stepper_t* stepper, uint8_t method);
uint8_t CANopen_SetHomingSpeeds(CANopen_Stepper_t* stepper, uint32_t fast, uint32_t slow);
uint8_t CANopen_SetHomingAcceleration(CANopen_Stepper_t* stepper, uint32_t acc);
uint8_t CANopen_StartHoming(CANopen_Stepper_t* stepper);
bool CANopen_IsHomingComplete(CANopen_Stepper_t* stepper);

// 电机参数设置
uint8_t CANopen_SetMotorCurrent(CANopen_Stepper_t* stepper, uint16_t current_mA);
uint8_t CANopen_SetSubdivision(CANopen_Stepper_t* stepper, uint16_t subdivision);

// 状态查询
int32_t CANopen_GetActualPosition(CANopen_Stepper_t* stepper);
int32_t CANopen_GetActualVelocity(CANopen_Stepper_t* stepper);
uint16_t CANopen_GetStatusWord(CANopen_Stepper_t* stepper);

// CAN消息处理
void CANopen_ProcessRxMessage(CANopen_Stepper_t* stepper, uint32_t rx_id, uint8_t* data);

// 周期性任务
void CANopen_Task(CANopen_Stepper_t* stepper);

// 辅助函数
const char* CANopen_GetStateName(DS402State_t state);
const char* CANopen_GetModeName(OperationMode_t mode);

#endif /* __CANOPEN_STEPPER_H */