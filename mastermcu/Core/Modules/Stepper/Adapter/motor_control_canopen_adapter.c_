#include "../motor_control_types.h"
#include "../CANopen/canopen_stepper.h"

// 全局CANopen步进电机实例
static CANopen_Stepper_t g_canopen_stepper;
static bool g_use_canopen = true;  // 使用CANopen模式

// 将Motor_t映射到CANopen_Stepper_t
static void Motor_UpdateFromCANopen(Motor_t* motor) {
    motor->current_position = g_canopen_stepper.actual_position;
    motor->target_position = g_canopen_stepper.target_position;
    motor->is_moving = !g_canopen_stepper.status_word.bits.target_reached;
    motor->speed = (uint16_t)(g_canopen_stepper.actual_velocity > 0 ? 
                              g_canopen_stepper.actual_velocity : -g_canopen_stepper.actual_velocity);
    
    // 映射状态
    if (g_canopen_stepper.status_word.bits.fault) {
        motor->status = MOTOR_STATUS_FAULT;
    } else if (g_canopen_stepper.state == STATE_OPERATION_ENABLED && motor->is_moving) {
        motor->status = MOTOR_STATUS_RUNNING;
    } else if (!g_canopen_stepper.is_connected) {
        motor->status = MOTOR_STATUS_TIMEOUT;
    } else {
        motor->status = MOTOR_STATUS_IDLE;
    }
    
    motor->last_response_time = g_canopen_stepper.last_heartbeat_time;
}

// 初始化电机（CANopen模式）
void Motor_Init_CANopen(Motor_t* motor, uint16_t motor_id) {
    motor->motor_can_id = motor_id;
    motor->current_position = 0;
    motor->target_position = 0;
    motor->is_moving = false;
    motor->speed = 1000;
    motor->status = MOTOR_STATUS_IDLE;
    motor->last_command_time = 0;
    motor->last_response_time = HAL_GetTick();
    
    if (g_use_canopen) {
        // 初始化CANopen步进电机
        CANopen_Stepper_Init(&g_canopen_stepper, (uint8_t)motor_id);
        
        // 设置默认参数
        CANopen_SetSubdivision(&g_canopen_stepper, 4000);      // 4000脉冲/圈
        CANopen_SetMotorCurrent(&g_canopen_stepper, 3000);     // 3A
        
        // 故障复位
        CANopen_FaultReset(&g_canopen_stepper);
        HAL_Delay(100);
        
        // 使能运行
        CANopen_EnableOperation(&g_canopen_stepper);
        HAL_Delay(100);
        
        // 默认使用位置模式
        CANopen_SetOperationMode(&g_canopen_stepper, MODE_PROFILE_POSITION);
        HAL_Delay(50);
        
        // 设置默认运动参数
        CANopen_SetProfileVelocity(&g_canopen_stepper, 8000);
        CANopen_SetProfileAcceleration(&g_canopen_stepper, 2000, 2000);
    }
    
    // 启动CAN
    CAN1_UserFilterStart();
}

// 绝对位置移动（CANopen模式）
uint8_t Motor_MoveTo_CANopen(Motor_t* motor, int32_t position) {
    if (g_use_canopen) {
        // 确保在位置模式
        if (g_canopen_stepper.operation_mode != MODE_PROFILE_POSITION) {
            CANopen_SetOperationMode(&g_canopen_stepper, MODE_PROFILE_POSITION);
            HAL_Delay(50);
        }
        
        // 设置目标位置（绝对）
        CANopen_SetTargetPosition(&g_canopen_stepper, position, true);
        
        // 启动移动
        uint8_t result = CANopen_StartPositionMove(&g_canopen_stepper);
        
        motor->target_position = position;
        motor->is_moving = true;
        motor->last_command_time = HAL_GetTick();
        
        return result;
    } else {
        // 使用原有实现
        return Motor_MoveTo(motor, position);
    }
}

// 相对位置移动（CANopen模式）
uint8_t Motor_MoveSteps_CANopen(Motor_t* motor, int32_t steps) {
    if (g_use_canopen) {
        // 确保在位置模式
        if (g_canopen_stepper.operation_mode != MODE_PROFILE_POSITION) {
            CANopen_SetOperationMode(&g_canopen_stepper, MODE_PROFILE_POSITION);
            HAL_Delay(50);
        }
        
        // 设置目标位置（相对）
        CANopen_SetTargetPosition(&g_canopen_stepper, steps, false);
        
        // 启动移动
        uint8_t result = CANopen_StartPositionMove(&g_canopen_stepper);
        
        motor->target_position = motor->current_position + steps;
        motor->is_moving = true;
        motor->last_command_time = HAL_GetTick();
        
        return result;
    } else {
        // 使用原有实现
        return Motor_MoveSteps(motor, steps);
    }
}

// 设置速度（CANopen模式）
uint8_t Motor_SetSpeed_CANopen(Motor_t* motor, uint16_t speed) {
    if (g_use_canopen) {
        // 在位置模式下设置轮廓速度
        uint8_t result = CANopen_SetProfileVelocity(&g_canopen_stepper, (uint32_t)speed);
        
        motor->speed = speed;
        motor->last_command_time = HAL_GetTick();
        
        return result;
    } else {
        // 使用原有实现
        return Motor_SetSpeed(motor, speed);
    }
}

// 停止电机（CANopen模式）
uint8_t Motor_Stop_CANopen(Motor_t* motor) {
    if (g_use_canopen) {
        uint8_t result = CANopen_QuickStop(&g_canopen_stepper);
        
        motor->is_moving = false;
        motor->target_position = motor->current_position;
        motor->last_command_time = HAL_GetTick();
        
        // 恢复到运行使能状态
        HAL_Delay(100);
        CANopen_EnableOperation(&g_canopen_stepper);
        
        return result;
    } else {
        // 使用原有实现
        return Motor_Stop(motor);
    }
}

// 设置零点（CANopen模式）
uint8_t Motor_SetZero_CANopen(Motor_t* motor) {
    if (g_use_canopen) {
        // 使用方法35：当前位置设为原点
        CANopen_SetOperationMode(&g_canopen_stepper, MODE_HOMING);
        HAL_Delay(50);
        
        CANopen_SetHomingMethod(&g_canopen_stepper, 35);
        uint8_t result = CANopen_StartHoming(&g_canopen_stepper);
        
        motor->current_position = 0;
        g_canopen_stepper.actual_position = 0;
        
        // 切回位置模式
        HAL_Delay(100);
        CANopen_SetOperationMode(&g_canopen_stepper, MODE_PROFILE_POSITION);
        
        return result;
    } else {
        motor->current_position = 0;
        motor->target_position = 0;
        return 0;
    }
}

// 读取位置（CANopen模式）
uint8_t Motor_ReadPosition_CANopen(Motor_t* motor) {
    if (g_use_canopen) {
        return CANopen_SDO_Read(&g_canopen_stepper, OD_POSITION_ACTUAL, 0);
    } else {
        return Motor_ReadPosition(motor);
    }
}

// 读取状态（CANopen模式）
uint8_t Motor_ReadStatus_CANopen(Motor_t* motor) {
    if (g_use_canopen) {
        return CANopen_SDO_Read(&g_canopen_stepper, OD_STATUS_WORD, 0);
    } else {
        return Motor_ReadStatus(motor);
    }
}

// CAN接收处理（CANopen模式）
void Motor_ProcessRxMessage_CANopen(Motor_t* motor, uint16_t rx_id, uint8_t* data) {
    if (g_use_canopen) {
        CANopen_ProcessRxMessage(&g_canopen_stepper, rx_id, data);
        Motor_UpdateFromCANopen(motor);
    } else {
        Motor_ProcessRxMessage(motor, rx_id, data);
    }
}

// 任务处理（CANopen模式）
void Motor_Task_CANopen(Motor_t* motor) {
    if (g_use_canopen) {
        CANopen_Task(&g_canopen_stepper);
        Motor_UpdateFromCANopen(motor);
    } else {
        Motor_Task(motor);
    }
}

// 速度模式控制（新增）
uint8_t Motor_StartVelocityMode(Motor_t* motor, int32_t velocity) {
    if (!g_use_canopen) return 1;
    
    // 切换到速度模式
    CANopen_SetOperationMode(&g_canopen_stepper, MODE_PROFILE_VELOCITY);
    HAL_Delay(50);
    
    // 设置目标速度
    CANopen_SetTargetVelocity(&g_canopen_stepper, velocity);
    
    // 启动
    return CANopen_StartVelocityMode(&g_canopen_stepper);
}

// 回原点控制（新增）
uint8_t Motor_StartHoming(Motor_t* motor, uint8_t method) {
    if (!g_use_canopen) return 1;
    
    // 切换到回原模式
    CANopen_SetOperationMode(&g_canopen_stepper, MODE_HOMING);
    HAL_Delay(50);
    
    // 设置回原方法
    CANopen_SetHomingMethod(&g_canopen_stepper, method);
    CANopen_SetHomingSpeeds(&g_canopen_stepper, 4000, 2000);
    CANopen_SetHomingAcceleration(&g_canopen_stepper, 1000);
    
    // 启动回原
    return CANopen_StartHoming(&g_canopen_stepper);
}

// 检查回原是否完成
bool Motor_IsHomingComplete(Motor_t* motor) {
    if (!g_use_canopen) return false;
    return CANopen_IsHomingComplete(&g_canopen_stepper);
}

// 设置是否使用CANopen模式
void Motor_SetCANopenMode(bool use_canopen) {
    g_use_canopen = use_canopen;
}

// 获取CANopen步进电机实例（用于高级控制）
CANopen_Stepper_t* Motor_GetCANopenInstance(void) {
    return &g_canopen_stepper;
}

// 重定义原有函数（如果需要完全替换）
#ifdef USE_CANOPEN_ADAPTER
#undef Motor_Init
#undef Motor_MoveTo
#undef Motor_MoveSteps
#undef Motor_SetSpeed
#undef Motor_Stop
#undef Motor_SetZero
#undef Motor_ReadPosition
#undef Motor_ReadStatus
#undef Motor_ProcessRxMessage
#undef Motor_Task

#define Motor_Init              Motor_Init_CANopen
#define Motor_MoveTo            Motor_MoveTo_CANopen
#define Motor_MoveSteps         Motor_MoveSteps_CANopen
#define Motor_SetSpeed          Motor_SetSpeed_CANopen
#define Motor_Stop              Motor_Stop_CANopen
#define Motor_SetZero           Motor_SetZero_CANopen
#define Motor_ReadPosition      Motor_ReadPosition_CANopen
#define Motor_ReadStatus        Motor_ReadStatus_CANopen
#define Motor_ProcessRxMessage  Motor_ProcessRxMessage_CANopen
#define Motor_Task              Motor_Task_CANopen
#endif