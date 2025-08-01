#include "canopen_stepper.h"
#include "../../Global_bb/blackboard.h"

// 步进电机实例
static CANopen_Stepper_t g_stepper;

// 初始化示例
void Stepper_Example_Init(void) {
    // 初始化步进电机（节点ID=1）
    CANopen_Stepper_Init(&g_stepper, 1);
    
    // 设置基本参数
    CANopen_SetSubdivision(&g_stepper, 4000);      // 4000脉冲/圈
    CANopen_SetMotorCurrent(&g_stepper, 3000);     // 3A电流
    
    // 故障复位
    CANopen_FaultReset(&g_stepper);
    HAL_Delay(100);
    
    // 使能电机
    CANopen_EnableOperation(&g_stepper);
    HAL_Delay(100);
}

// 位置模式示例 - 绝对位置运动
void Stepper_Example_AbsoluteMove(void) {
    // 切换到位置模式
    CANopen_SetOperationMode(&g_stepper, MODE_PROFILE_POSITION);
    HAL_Delay(50);
    
    // 设置运动参数
    CANopen_SetProfileVelocity(&g_stepper, 8000);        // 8000脉冲/秒
    CANopen_SetProfileAcceleration(&g_stepper, 2000, 2000); // 加减速度2000脉冲/秒²
    
    // 设置目标位置（绝对位置30000脉冲）
    CANopen_SetTargetPosition(&g_stepper, 30000, true);
    
    // 启动运动
    CANopen_StartPositionMove(&g_stepper);
    
    // 等待到达目标位置
    while (!CANopen_IsTargetReached(&g_stepper)) {
        CANopen_Task(&g_stepper);
        HAL_Delay(10);
    }
}

// 位置模式示例 - 相对位置运动
void Stepper_Example_RelativeMove(void) {
    // 切换到位置模式
    CANopen_SetOperationMode(&g_stepper, MODE_PROFILE_POSITION);
    HAL_Delay(50);
    
    // 设置运动参数
    CANopen_SetProfileVelocity(&g_stepper, 8000);
    CANopen_SetProfileAcceleration(&g_stepper, 2000, 2000);
    
    // 设置目标位置（相对位置+30000脉冲）
    CANopen_SetTargetPosition(&g_stepper, 30000, false);
    
    // 启动运动
    CANopen_StartPositionMove(&g_stepper);
    
    // 等待到达目标位置
    while (!CANopen_IsTargetReached(&g_stepper)) {
        CANopen_Task(&g_stepper);
        HAL_Delay(10);
    }
}

// 速度模式示例
void Stepper_Example_VelocityMode(void) {
    // 切换到速度模式
    CANopen_SetOperationMode(&g_stepper, MODE_PROFILE_VELOCITY);
    HAL_Delay(50);
    
    // 设置加减速度
    CANopen_SetProfileAcceleration(&g_stepper, 2000, 2000);
    
    // 设置目标速度（8000脉冲/秒，正向）
    CANopen_SetTargetVelocity(&g_stepper, 8000);
    
    // 启动速度模式
    CANopen_StartVelocityMode(&g_stepper);
    
    // 运行5秒
    HAL_Delay(5000);
    
    // 改变速度方向（-1000脉冲/秒，反向）
    CANopen_SetTargetVelocity(&g_stepper, -1000);
    
    // 再运行3秒
    HAL_Delay(3000);
    
    // 停止
    CANopen_HaltVelocityMode(&g_stepper);
}

// 回原点模式示例
void Stepper_Example_Homing(void) {
    // 切换到回原模式
    CANopen_SetOperationMode(&g_stepper, MODE_HOMING);
    HAL_Delay(50);
    
    // 设置回原参数
    CANopen_SetHomingMethod(&g_stepper, 24);         // 方法24
    CANopen_SetHomingSpeeds(&g_stepper, 4000, 2000); // 快速4000，慢速2000
    CANopen_SetHomingAcceleration(&g_stepper, 1000); // 加速度1000
    
    // 启动回原
    CANopen_StartHoming(&g_stepper);
    
    // 等待回原完成
    while (!CANopen_IsHomingComplete(&g_stepper)) {
        CANopen_Task(&g_stepper);
        HAL_Delay(10);
        
        // 检查状态字bit13（回原错误）
        if (g_stepper.status_word.bits.op_mode_specific2) {
            // 回原错误处理
            break;
        }
    }
}

// 中断中立即停止
void Stepper_Example_QuickStop(void) {
    CANopen_QuickStop(&g_stepper);
}

// 减速停止（速度模式）
void Stepper_Example_DecelerateStop(void) {
    CANopen_HaltVelocityMode(&g_stepper);
}

// 电梯应用示例 - 移动到指定楼层
void Stepper_MoveToFloor(uint8_t floor) {
    int32_t target_position = 0;
    
    // 根据楼层计算目标位置
    switch(floor) {
        case 0: // 1楼
            target_position = 0;
            break;
        case 1: // 2楼
            target_position = 10000;
            break;
        case 2: // 3楼
            target_position = 20000;
            break;
        default:
            return;
    }
    
    // 切换到位置模式
    CANopen_SetOperationMode(&g_stepper, MODE_PROFILE_POSITION);
    HAL_Delay(50);
    
    // 设置运动参数
    CANopen_SetProfileVelocity(&g_stepper, 8000);
    CANopen_SetProfileAcceleration(&g_stepper, 2000, 2000);
    
    // 设置目标位置（绝对位置）
    CANopen_SetTargetPosition(&g_stepper, target_position, true);
    
    // 启动运动
    CANopen_StartPositionMove(&g_stepper);
}

// 电梯应用 - 检查是否到达楼层
bool Stepper_IsFloorReached(void) {
    return CANopen_IsTargetReached(&g_stepper);
}

// 电梯应用 - 获取当前位置
int32_t Stepper_GetCurrentPosition(void) {
    return CANopen_GetActualPosition(&g_stepper);
}

// 电梯应用 - 紧急停止
void Stepper_EmergencyStop(void) {
    CANopen_QuickStop(&g_stepper);
}

// CAN接收中断回调（需要在CAN中断中调用）
void Stepper_CAN_RxCallback(uint32_t rx_id, uint8_t* data) {
    CANopen_ProcessRxMessage(&g_stepper, rx_id, data);
}

// 周期性任务（需要在主循环中定期调用）
void Stepper_PeriodicTask(void) {
    CANopen_Task(&g_stepper);
    
    // 更新黑板数据
    g_blackboard.motor_position = g_stepper.actual_position;
    g_blackboard.motor_velocity = g_stepper.actual_velocity;
    g_blackboard.motor_connected = g_stepper.is_connected;
    
    // 根据位置计算当前楼层
    if (g_stepper.actual_position < 5000) {
        g_blackboard.current_floor = 0; // 1楼
    } else if (g_stepper.actual_position < 15000) {
        g_blackboard.current_floor = 1; // 2楼
    } else {
        g_blackboard.current_floor = 2; // 3楼
    }
}