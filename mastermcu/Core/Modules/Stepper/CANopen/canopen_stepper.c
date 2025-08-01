#include "canopen_stepper.h"
#include <string.h>

// SDO命令说明符定义
#define SDO_CS_DOWNLOAD_INIT_1BYTE  0x2F
#define SDO_CS_DOWNLOAD_INIT_2BYTE  0x2B
#define SDO_CS_DOWNLOAD_INIT_4BYTE  0x23
#define SDO_CS_UPLOAD_INIT          0x40
#define SDO_CS_DOWNLOAD_RESP        0x60
#define SDO_CS_UPLOAD_RESP_1BYTE    0x4F
#define SDO_CS_UPLOAD_RESP_2BYTE    0x4B
#define SDO_CS_UPLOAD_RESP_4BYTE    0x43
#define SDO_CS_ABORT                0x80

// 初始化步进电机控制器
void CANopen_Stepper_Init(CANopen_Stepper_t* stepper, uint8_t node_id) {
    memset(stepper, 0, sizeof(CANopen_Stepper_t));
    
    stepper->node_id = node_id;
    stepper->rx_cob_id = 0x600 + node_id;
    stepper->tx_cob_id = 0x580 + node_id;
    
    stepper->state = STATE_NOT_READY;
    stepper->operation_mode = MODE_NO_MODE;
    
    // 默认参数
    stepper->motor_subdivision = 4000;  // 4000脉冲/圈
    stepper->motor_rated_current = 3000; // 3000mA
    stepper->profile_velocity = 8000;    // 8000脉冲/秒
    stepper->profile_acc = 2000;         // 2000脉冲/秒²
    stepper->profile_dec = 2000;         // 2000脉冲/秒²
    
    stepper->last_sdo_time = HAL_GetTick();
    stepper->last_heartbeat_time = HAL_GetTick();
    stepper->is_connected = false;
}

// SDO写入
uint8_t CANopen_SDO_Write(CANopen_Stepper_t* stepper, uint16_t index, uint8_t subindex, 
                         void* data, uint8_t size) {
    uint8_t tx_data[8] = {0};
    
    // 设置索引和子索引
    tx_data[1] = index & 0xFF;
    tx_data[2] = (index >> 8) & 0xFF;
    tx_data[3] = subindex;
    
    // 根据数据大小设置命令说明符和数据
    switch(size) {
        case 1:
            tx_data[0] = SDO_CS_DOWNLOAD_INIT_1BYTE;
            tx_data[4] = *((uint8_t*)data);
            break;
        case 2:
            tx_data[0] = SDO_CS_DOWNLOAD_INIT_2BYTE;
            tx_data[4] = ((uint8_t*)data)[0];
            tx_data[5] = ((uint8_t*)data)[1];
            break;
        case 4:
            tx_data[0] = SDO_CS_DOWNLOAD_INIT_4BYTE;
            memcpy(&tx_data[4], data, 4);
            break;
        default:
            return 1; // 不支持的数据大小
    }
    
    stepper->last_sdo_time = HAL_GetTick();
    return CAN1_Send_Num(stepper->rx_cob_id, tx_data);
}

// SDO读取
uint8_t CANopen_SDO_Read(CANopen_Stepper_t* stepper, uint16_t index, uint8_t subindex) {
    uint8_t tx_data[8] = {0};
    
    tx_data[0] = SDO_CS_UPLOAD_INIT;
    tx_data[1] = index & 0xFF;
    tx_data[2] = (index >> 8) & 0xFF;
    tx_data[3] = subindex;
    
    stepper->last_sdo_time = HAL_GetTick();
    return CAN1_Send_Num(stepper->rx_cob_id, tx_data);
}

// 设置DS402状态机状态
uint8_t CANopen_SetState(CANopen_Stepper_t* stepper, DS402State_t target_state) {
    ControlWord_t cw = {0};
    
    // 根据目标状态设置控制字
    switch(target_state) {
        case STATE_SWITCH_ON_DISABLED:
            cw.value = 0x0000;  // 禁用
            break;
            
        case STATE_READY_TO_SWITCH_ON:
            cw.value = 0x0006;  // 关机
            break;
            
        case STATE_SWITCHED_ON:
            cw.value = 0x0007;  // 开机
            break;
            
        case STATE_OPERATION_ENABLED:
            cw.value = 0x000F;  // 使能运行
            break;
            
        case STATE_QUICK_STOP_ACTIVE:
            cw.value = 0x0002;  // 快速停止
            break;
            
        default:
            return 1;
    }
    
    stepper->control_word = cw;
    return CANopen_SDO_Write(stepper, OD_CONTROL_WORD, 0, &cw.value, 2);
}

// 获取当前状态
DS402State_t CANopen_GetState(CANopen_Stepper_t* stepper) {
    StatusWord_t sw = stepper->status_word;
    
    // 根据状态字判断当前状态
    if (!sw.bits.ready_to_switch_on && !sw.bits.switched_on && 
        !sw.bits.operation_enabled && !sw.bits.fault) {
        return STATE_NOT_READY;
    }
    
    if (!sw.bits.ready_to_switch_on && !sw.bits.switched_on && 
        !sw.bits.operation_enabled && sw.bits.switch_on_disabled && !sw.bits.fault) {
        return STATE_SWITCH_ON_DISABLED;
    }
    
    if (sw.bits.ready_to_switch_on && !sw.bits.switched_on && 
        !sw.bits.operation_enabled && !sw.bits.switch_on_disabled && 
        sw.bits.quick_stop && !sw.bits.fault) {
        return STATE_READY_TO_SWITCH_ON;
    }
    
    if (sw.bits.ready_to_switch_on && sw.bits.switched_on && 
        !sw.bits.operation_enabled && !sw.bits.switch_on_disabled && 
        sw.bits.quick_stop && !sw.bits.fault) {
        return STATE_SWITCHED_ON;
    }
    
    if (sw.bits.ready_to_switch_on && sw.bits.switched_on && 
        sw.bits.operation_enabled && !sw.bits.switch_on_disabled && 
        sw.bits.quick_stop && !sw.bits.fault) {
        return STATE_OPERATION_ENABLED;
    }
    
    if (sw.bits.ready_to_switch_on && sw.bits.switched_on && 
        sw.bits.operation_enabled && !sw.bits.switch_on_disabled && 
        !sw.bits.quick_stop && !sw.bits.fault) {
        return STATE_QUICK_STOP_ACTIVE;
    }
    
    if (sw.bits.fault) {
        return STATE_FAULT;
    }
    
    return STATE_NOT_READY;
}

// 使能运行
uint8_t CANopen_EnableOperation(CANopen_Stepper_t* stepper) {
    uint8_t result;
    
    // 先切换到开机状态
    result = CANopen_SetState(stepper, STATE_SWITCHED_ON);
    if (result != 0) return result;
    
    HAL_Delay(10); // 等待状态切换
    
    // 再切换到运行使能状态
    return CANopen_SetState(stepper, STATE_OPERATION_ENABLED);
}

// 禁用运行
uint8_t CANopen_DisableOperation(CANopen_Stepper_t* stepper) {
    return CANopen_SetState(stepper, STATE_SWITCHED_ON);
}

// 快速停止
uint8_t CANopen_QuickStop(CANopen_Stepper_t* stepper) {
    ControlWord_t cw = stepper->control_word;
    cw.bits.quick_stop = 0;  // 快速停止是0有效
    stepper->control_word = cw;
    return CANopen_SDO_Write(stepper, OD_CONTROL_WORD, 0, &cw.value, 2);
}

// 故障复位
uint8_t CANopen_FaultReset(CANopen_Stepper_t* stepper) {
    ControlWord_t cw = stepper->control_word;
    cw.bits.fault_reset = 1;
    stepper->control_word = cw;
    
    uint8_t result = CANopen_SDO_Write(stepper, OD_CONTROL_WORD, 0, &cw.value, 2);
    
    // 清除故障复位位
    HAL_Delay(10);
    cw.bits.fault_reset = 0;
    stepper->control_word = cw;
    CANopen_SDO_Write(stepper, OD_CONTROL_WORD, 0, &cw.value, 2);
    
    return result;
}

// 设置运行模式
uint8_t CANopen_SetOperationMode(CANopen_Stepper_t* stepper, OperationMode_t mode) {
    int8_t mode_value = (int8_t)mode;
    stepper->operation_mode = mode;
    return CANopen_SDO_Write(stepper, OD_OPERATION_MODE, 0, &mode_value, 1);
}

// 获取运行模式
OperationMode_t CANopen_GetOperationMode(CANopen_Stepper_t* stepper) {
    // 发送读取请求
    CANopen_SDO_Read(stepper, OD_OPERATION_MODE_DISP, 0);
    return stepper->operation_mode;
}

// 设置目标位置
uint8_t CANopen_SetTargetPosition(CANopen_Stepper_t* stepper, int32_t position, bool absolute) {
    // 设置绝对/相对模式
    ControlWord_t cw = stepper->control_word;
    cw.bits.abs_rel = absolute ? 0 : 1;
    stepper->control_word = cw;
    
    // 设置目标位置
    stepper->target_position = position;
    return CANopen_SDO_Write(stepper, OD_TARGET_POSITION, 0, &position, 4);
}

// 设置轮廓速度
uint8_t CANopen_SetProfileVelocity(CANopen_Stepper_t* stepper, uint32_t velocity) {
    stepper->profile_velocity = velocity;
    return CANopen_SDO_Write(stepper, OD_PROFILE_VELOCITY, 0, &velocity, 4);
}

// 设置轮廓加减速度
uint8_t CANopen_SetProfileAcceleration(CANopen_Stepper_t* stepper, uint32_t acc, uint32_t dec) {
    uint8_t result;
    
    stepper->profile_acc = acc;
    result = CANopen_SDO_Write(stepper, OD_PROFILE_ACCELERATION, 0, &acc, 4);
    if (result != 0) return result;
    
    stepper->profile_dec = dec;
    return CANopen_SDO_Write(stepper, OD_PROFILE_DECELERATION, 0, &dec, 4);
}

// 启动位置移动
uint8_t CANopen_StartPositionMove(CANopen_Stepper_t* stepper) {
    ControlWord_t cw = stepper->control_word;
    
    // 确保在运行使能状态
    if (stepper->state != STATE_OPERATION_ENABLED) {
        CANopen_EnableOperation(stepper);
        HAL_Delay(50);
    }
    
    // 清除新设定点位
    cw.bits.new_setpoint = 0;
    stepper->control_word = cw;
    CANopen_SDO_Write(stepper, OD_CONTROL_WORD, 0, &cw.value, 2);
    
    HAL_Delay(10);
    
    // 触发新设定点
    cw.bits.new_setpoint = 1;
    stepper->control_word = cw;
    return CANopen_SDO_Write(stepper, OD_CONTROL_WORD, 0, &cw.value, 2);
}

// 检查是否到达目标位置
bool CANopen_IsTargetReached(CANopen_Stepper_t* stepper) {
    return stepper->status_word.bits.target_reached;
}

// 设置目标速度
uint8_t CANopen_SetTargetVelocity(CANopen_Stepper_t* stepper, int32_t velocity) {
    stepper->target_velocity = velocity;
    return CANopen_SDO_Write(stepper, OD_TARGET_VELOCITY, 0, &velocity, 4);
}

// 启动速度模式
uint8_t CANopen_StartVelocityMode(CANopen_Stepper_t* stepper) {
    // 确保在运行使能状态
    if (stepper->state != STATE_OPERATION_ENABLED) {
        return CANopen_EnableOperation(stepper);
    }
    return 0;
}

// 停止速度模式
uint8_t CANopen_HaltVelocityMode(CANopen_Stepper_t* stepper) {
    ControlWord_t cw = stepper->control_word;
    cw.bits.halt = 1;
    stepper->control_word = cw;
    return CANopen_SDO_Write(stepper, OD_CONTROL_WORD, 0, &cw.value, 2);
}

// 设置回原方式
uint8_t CANopen_SetHomingMethod(CANopen_Stepper_t* stepper, uint8_t method) {
    stepper->homing_method = method;
    return CANopen_SDO_Write(stepper, OD_HOMING_METHOD, 0, &method, 1);
}

// 设置回原速度
uint8_t CANopen_SetHomingSpeeds(CANopen_Stepper_t* stepper, uint32_t fast, uint32_t slow) {
    uint8_t result;
    
    stepper->homing_speed_fast = fast;
    result = CANopen_SDO_Write(stepper, OD_HOMING_SPEEDS, 1, &fast, 4);
    if (result != 0) return result;
    
    stepper->homing_speed_slow = slow;
    return CANopen_SDO_Write(stepper, OD_HOMING_SPEEDS, 2, &slow, 4);
}

// 设置回原加速度
uint8_t CANopen_SetHomingAcceleration(CANopen_Stepper_t* stepper, uint32_t acc) {
    stepper->homing_acceleration = acc;
    return CANopen_SDO_Write(stepper, OD_HOMING_ACCELERATION, 0, &acc, 4);
}

// 启动回原
uint8_t CANopen_StartHoming(CANopen_Stepper_t* stepper) {
    ControlWord_t cw = stepper->control_word;
    
    // 确保在运行使能状态
    if (stepper->state != STATE_OPERATION_ENABLED) {
        CANopen_EnableOperation(stepper);
        HAL_Delay(50);
    }
    
    // 启动回原
    cw.bits.new_setpoint = 1;  // 在回原模式下，bit4用于启动回原
    stepper->control_word = cw;
    return CANopen_SDO_Write(stepper, OD_CONTROL_WORD, 0, &cw.value, 2);
}

// 检查回原是否完成
bool CANopen_IsHomingComplete(CANopen_Stepper_t* stepper) {
    // 在回原模式下，bit12表示回原完成
    return (stepper->status_word.bits.op_mode_specific1 != 0);
}

// 设置电机电流
uint8_t CANopen_SetMotorCurrent(CANopen_Stepper_t* stepper, uint16_t current_mA) {
    stepper->motor_rated_current = current_mA;
    return CANopen_SDO_Write(stepper, OD_MOTOR_RATED_CURRENT, 0, &current_mA, 2);
}

// 设置细分
uint8_t CANopen_SetSubdivision(CANopen_Stepper_t* stepper, uint16_t subdivision) {
    stepper->motor_subdivision = subdivision;
    return CANopen_SDO_Write(stepper, OD_MOTOR_SUBDIVISION, 0, &subdivision, 2);
}

// 获取实际位置
int32_t CANopen_GetActualPosition(CANopen_Stepper_t* stepper) {
    CANopen_SDO_Read(stepper, OD_POSITION_ACTUAL, 0);
    return stepper->actual_position;
}

// 获取实际速度
int32_t CANopen_GetActualVelocity(CANopen_Stepper_t* stepper) {
    CANopen_SDO_Read(stepper, OD_VELOCITY_ACTUAL, 0);
    return stepper->actual_velocity;
}

// 获取状态字
uint16_t CANopen_GetStatusWord(CANopen_Stepper_t* stepper) {
    CANopen_SDO_Read(stepper, OD_STATUS_WORD, 0);
    return stepper->status_word.value;
}

// 处理接收到的CAN消息
void CANopen_ProcessRxMessage(CANopen_Stepper_t* stepper, uint32_t rx_id, uint8_t* data) {
    // 检查是否是SDO响应
    if (rx_id == stepper->tx_cob_id) {
        uint8_t cs = data[0];
        uint16_t index = data[1] | (data[2] << 8);
        uint8_t subindex = data[3];
        
        // 处理SDO响应
        if (cs == SDO_CS_DOWNLOAD_RESP) {
            // SDO写入成功确认
        } else if ((cs & 0xE0) == 0x40) {
            // SDO读取响应
            switch(index) {
                case OD_STATUS_WORD:
                    stepper->status_word.value = data[4] | (data[5] << 8);
                    stepper->state = CANopen_GetState(stepper);
                    break;
                    
                case OD_POSITION_ACTUAL:
                    memcpy(&stepper->actual_position, &data[4], 4);
                    break;
                    
                case OD_VELOCITY_ACTUAL:
                    memcpy(&stepper->actual_velocity, &data[4], 4);
                    break;
                    
                case OD_OPERATION_MODE_DISP:
                    stepper->operation_mode = (OperationMode_t)data[4];
                    break;
            }
        }
        
        stepper->is_connected = true;
        stepper->last_heartbeat_time = HAL_GetTick();
    }
}

// 周期性任务
void CANopen_Task(CANopen_Stepper_t* stepper) {
    uint32_t current_time = HAL_GetTick();
    
    // 定期读取状态字
    if (current_time - stepper->last_sdo_time > 50) {
        CANopen_GetStatusWord(stepper);
        stepper->last_sdo_time = current_time;
    }
    
    // 检查连接超时
    if (current_time - stepper->last_heartbeat_time > 1000) {
        stepper->is_connected = false;
    }
}

// 获取状态名称
const char* CANopen_GetStateName(DS402State_t state) {
    switch(state) {
        case STATE_NOT_READY: return "Not Ready";
        case STATE_SWITCH_ON_DISABLED: return "Switch On Disabled";
        case STATE_READY_TO_SWITCH_ON: return "Ready to Switch On";
        case STATE_SWITCHED_ON: return "Switched On";
        case STATE_OPERATION_ENABLED: return "Operation Enabled";
        case STATE_QUICK_STOP_ACTIVE: return "Quick Stop Active";
        case STATE_FAULT_REACTION_ACTIVE: return "Fault Reaction Active";
        case STATE_FAULT: return "Fault";
        default: return "Unknown";
    }
}

// 获取模式名称
const char* CANopen_GetModeName(OperationMode_t mode) {
    switch(mode) {
        case MODE_NO_MODE: return "No Mode";
        case MODE_PROFILE_POSITION: return "Profile Position";
        case MODE_VELOCITY: return "Velocity";
        case MODE_PROFILE_VELOCITY: return "Profile Velocity";
        case MODE_PROFILE_TORQUE: return "Profile Torque";
        case MODE_HOMING: return "Homing";
        case MODE_INTERPOLATED_POSITION: return "Interpolated Position";
        default: return "Unknown";
    }
}