#include "elevator_fsm.h"
#include "motor_control.h"
#include "motor_advanced.h"
#include "rs485_protocol.h"
#include "calibration.h"
#include <stdio.h>

// 外部变量
extern Motor_t motor;
extern MotorAdvanced_t motor_adv;
extern Calibration_t calibration;

// 前向声明
static void update_target_floor(void);
static void check_door_timeout(void);
static void handle_error(void);

// 状态处理函数表
static StateHandler_t state_handlers[] = {
    [ELEVATOR_IDLE] = FSM_StateIdle,
    [ELEVATOR_MOVING_UP] = FSM_StateMovingUp,
    [ELEVATOR_MOVING_DOWN] = FSM_StateMovingDown,
    [ELEVATOR_DOOR_OPENING] = FSM_StateDoorOpening,
    [ELEVATOR_DOOR_OPEN] = FSM_StateDoorOpen,
    [ELEVATOR_DOOR_CLOSING] = FSM_StateDoorClosing,
    [ELEVATOR_DOOR_CLOSED] = FSM_StateDoorClosed,
    [ELEVATOR_EMERGENCY_STOP] = FSM_StateEmergencyStop,
    [ELEVATOR_CALIBRATING] = FSM_StateCalibrating,
    [ELEVATOR_FAULT] = FSM_StateFault
};

// 状态转换表
static const StateTransition_t transitions[] = {
    // 从校准状态
    {ELEVATOR_CALIBRATING, ELEVATOR_IDLE, FSM_CalibrationComplete, EVENT_CALIBRATION_DONE},
    
    // 从空闲状态
    {ELEVATOR_IDLE, ELEVATOR_DOOR_OPENING, FSM_IsAtTargetFloor, EVENT_NONE},
    {ELEVATOR_IDLE, ELEVATOR_MOVING_UP, FSM_HasCallsAbove, EVENT_NONE},
    {ELEVATOR_IDLE, ELEVATOR_MOVING_DOWN, FSM_HasCallsBelow, EVENT_NONE},
    
    // 从上行状态
    {ELEVATOR_MOVING_UP, ELEVATOR_DOOR_OPENING, FSM_ShouldStopAtFloor, EVENT_POSITION_REACHED},
    {ELEVATOR_MOVING_UP, ELEVATOR_EMERGENCY_STOP, FSM_HasEmergency, EVENT_EMERGENCY_STOP},
    
    // 从下行状态
    {ELEVATOR_MOVING_DOWN, ELEVATOR_DOOR_OPENING, FSM_ShouldStopAtFloor, EVENT_POSITION_REACHED},
    {ELEVATOR_MOVING_DOWN, ELEVATOR_EMERGENCY_STOP, FSM_HasEmergency, EVENT_EMERGENCY_STOP},
    
    // 门控制状态转换
    {ELEVATOR_DOOR_OPENING, ELEVATOR_DOOR_OPEN, FSM_DoorOpenComplete, EVENT_DOOR_OPENED},
    {ELEVATOR_DOOR_OPEN, ELEVATOR_DOOR_CLOSING, NULL, EVENT_NONE},  // 超时或按钮触发
    {ELEVATOR_DOOR_CLOSING, ELEVATOR_DOOR_CLOSED, FSM_DoorCloseComplete, EVENT_DOOR_CLOSED},
    {ELEVATOR_DOOR_CLOSED, ELEVATOR_IDLE, NULL, EVENT_NONE},
    
    // 紧急停止和故障处理
    {ELEVATOR_EMERGENCY_STOP, ELEVATOR_IDLE, FSM_CanRecoverFromFault, EVENT_NONE},
    {ELEVATOR_FAULT, ELEVATOR_CALIBRATING, FSM_CanRecoverFromFault, EVENT_NONE},
};

static const int num_transitions = sizeof(transitions) / sizeof(transitions[0]);

// 门控制定时器
static uint32_t door_timer = 0;
static const uint32_t DOOR_OPEN_TIME = 3000;  // 门保持开启时间

// FSM初始化
void FSM_Init(void) {
    Blackboard_Init();
    FSM_StartCalibration();
}

// FSM主处理循环
void FSM_Process(void) {
    ElevatorState_t current_state = Blackboard_GetState();
    
    // 处理事件队列
    Event_t event;
    while (Blackboard_PopEvent(&event)) {
        FSM_HandleEvent(&event);
    }
    
    // 检查状态转换
    for (int i = 0; i < num_transitions; i++) {
        if (transitions[i].from_state == current_state) {
            bool should_transition = false;
            
            // 检查转换条件
            if (transitions[i].condition != NULL) {
                should_transition = transitions[i].condition();
            } else {
                should_transition = true;  // 无条件转换
            }
            
            if (should_transition) {
                Blackboard_SetState(transitions[i].to_state);
                printf("State transition: %d -> %d\n", current_state, transitions[i].to_state);
                break;
            }
        }
    }
    
    // 执行当前状态处理函数
    current_state = Blackboard_GetState();
    if (current_state < sizeof(state_handlers) / sizeof(state_handlers[0])) {
        if (state_handlers[current_state] != NULL) {
            state_handlers[current_state]();
        }
    }
}

// 事件处理
void FSM_HandleEvent(Event_t* event) {
    g_blackboard.timestamp = HAL_GetTick();
    
    switch (event->type) {
        case EVENT_TARGET_UPDATED:
            // 目标更新，检查同方向和安全
            if (g_blackboard.state == ELEVATOR_IDLE && Blackboard_IsSafeToMove()) {
                // 重新评估目标
                update_target_floor();
            }
            break;
            
        case EVENT_FLOOR_REACHED:
            // 到达楼层
            g_blackboard.motor_running = false;
            g_blackboard.current_floor = event->floor;
            Blackboard_ClearPendingCall(event->floor + 1);
            
            if (event->floor == g_blackboard.target_floor) {
                Blackboard_PushEvent(EVENT_OPEN_DOOR, event->floor);
            }
            break;
            
        case EVENT_POSITION_ADJUST:
            // 位置调整
            g_blackboard.position_offset = event->data[0] | (event->data[1] << 8);
            break;
            
        case EVENT_DOOR_OPENED:
            g_blackboard.door_open_time = HAL_GetTick();
            break;
            
        case EVENT_SYNC_TIMEOUT:
            g_blackboard.rs485_sync_delay_flag = true;
            break;
            
        case EVENT_ERROR:
            handle_error();
            break;
            
        case EVENT_SENSOR_TRIGGERED:
            // 更新当前楼层位置
            g_blackboard.sensors.sensor_triggered[event->floor] = true;
            g_blackboard.sensors.last_trigger_time = event->timestamp;
            // 位置校准
            Blackboard_UpdateMotorPosition(Blackboard_FloorToPosition(event->floor));
            break;
            
        case EVENT_EMERGENCY_STOP:
            g_blackboard.emergency_stop = true;
            g_blackboard.motor_running = false;
            break;
            
        default:
            break;
    }
}

// 状态处理函数实现
void FSM_StateIdle(void) {
    // 检查是否有待处理的呼叫
    if (Blackboard_HasPendingCalls()) {
        uint8_t target = Blackboard_GetNextTargetFloor();
        g_blackboard.target_floor = target;
        
        if (target > g_blackboard.current_floor) {
            g_blackboard.dir = DIR_UP;
            g_blackboard.motor_running = true;
            Blackboard_PushEvent(EVENT_START_MOVING, target);
            Blackboard_SetState(ELEVATOR_MOVING_UP);
        } else if (target < g_blackboard.current_floor) {
            g_blackboard.dir = DIR_DOWN;
            g_blackboard.motor_running = true;
            Blackboard_PushEvent(EVENT_START_MOVING, target);
            Blackboard_SetState(ELEVATOR_MOVING_DOWN);
        } else {
            // 已在目标楼层，开门
            Blackboard_PushEvent(EVENT_OPEN_DOOR, target);
        }
    }
    
    // 检查门超时
    check_door_timeout();
}

void FSM_StateMovingUp(void) {
    // 监控电机状态
    if (!g_blackboard.motor.is_moving) {
        // 电机意外停止
        Blackboard_PushEvent(EVENT_MOTOR_FAULT, g_blackboard.current_floor);
        Blackboard_SetState(ELEVATOR_FAULT);
    }
    
    // 检查是否需要在当前楼层停止
    if (FSM_ShouldStopAtFloor()) {
        FSM_StopAtFloor();
    }
}

void FSM_StateMovingDown(void) {
    // 监控电机状态
    if (!g_blackboard.motor.is_moving) {
        // 电机意外停止
        Blackboard_PushEvent(EVENT_MOTOR_FAULT, g_blackboard.current_floor);
        Blackboard_SetState(ELEVATOR_FAULT);
    }
    
    // 检查是否需要在当前楼层停止
    if (FSM_ShouldStopAtFloor()) {
        FSM_StopAtFloor();
    }
}

void FSM_StateDoorOpening(void) {
    static bool opening_started = false;
    
    if (!opening_started) {
        FSM_OpenDoor();
        opening_started = true;
        door_timer = HAL_GetTick();
    }
    
    // 检查超时
    if (HAL_GetTick() - door_timer > 5000) {
        // 开门超时
        Blackboard_SetState(ELEVATOR_FAULT);
        opening_started = false;
    }
    
    if (g_blackboard.door.is_open) {
        opening_started = false;
    }
}

void FSM_StateDoorOpen(void) {
    static bool timer_started = false;
    
    if (!timer_started) {
        door_timer = HAL_GetTick();
        timer_started = true;
        // 清除当前楼层的呼叫
        FSM_ClearFloorCalls(g_blackboard.current_floor);
    }
    
    // 门保持开启一段时间
    if (HAL_GetTick() - door_timer > DOOR_OPEN_TIME) {
        Blackboard_SetState(ELEVATOR_DOOR_CLOSING);
        timer_started = false;
    }
}

void FSM_StateDoorClosing(void) {
    static bool closing_started = false;
    
    if (!closing_started) {
        FSM_CloseDoor();
        closing_started = true;
        door_timer = HAL_GetTick();
    }
    
    // 检查超时
    if (HAL_GetTick() - door_timer > 5000) {
        // 关门超时
        Blackboard_SetState(ELEVATOR_FAULT);
        closing_started = false;
    }
    
    if (g_blackboard.door.is_closed) {
        closing_started = false;
    }
}

void FSM_StateDoorClosed(void) {
    // 门已关闭，准备转入空闲状态
    Blackboard_SetState(ELEVATOR_IDLE);
}

void FSM_StateEmergencyStop(void) {
    // 紧急停止状态
    Motor_Stop(&motor);
    
    // 等待紧急停止解除
    if (!g_blackboard.emergency_stop) {
        Blackboard_SetState(ELEVATOR_CALIBRATING);
    }
}

void FSM_StateCalibrating(void) {
    static bool calibration_started = false;
    static uint32_t calibration_timer = 0;
    
    if (!calibration_started) {
        FSM_StartCalibration();
        calibration_started = true;
        calibration_timer = HAL_GetTick();
    }
    
    // 等待校准完成或超时
    if (g_blackboard.motor.is_calibrated) {
        calibration_started = false;
        Blackboard_PushEvent(EVENT_CALIBRATION_DONE, 0);
    } else if (HAL_GetTick() - calibration_timer > 30000) {
        // 校准超时
        calibration_started = false;
        Blackboard_SetState(ELEVATOR_FAULT);
    }
}

void FSM_StateFault(void) {
    // 故障状态处理
    Motor_Stop(&motor);
    
    // 尝试恢复
    static uint32_t fault_timer = 0;
    if (fault_timer == 0) {
        fault_timer = HAL_GetTick();
    }
    
    if (HAL_GetTick() - fault_timer > 5000) {
        // 5秒后尝试恢复
        if (FSM_CanRecoverFromFault()) {
            fault_timer = 0;
            Blackboard_SetState(ELEVATOR_CALIBRATING);
        }
    }
}

// 转换条件函数实现
bool FSM_CanStartMoving(void) {
    return Blackboard_IsSafeToMove() && Blackboard_HasPendingCalls();
}

bool FSM_ShouldStopAtFloor(void) {
    uint8_t current = g_blackboard.current_floor;
    bool going_up = (g_blackboard.state == ELEVATOR_MOVING_UP);
    
    // 检查是否到达目标楼层
    if (current == g_blackboard.target_floor) {
        return true;
    }
    
    // 检查是否需要在当前楼层停止
    if (going_up) {
        return g_blackboard.floors[current].call_up || 
               g_blackboard.floors[current].floor_request;
    } else {
        return g_blackboard.floors[current].call_down || 
               g_blackboard.floors[current].floor_request;
    }
}

bool FSM_DoorOpenComplete(void) {
    return g_blackboard.door.is_open;
}

bool FSM_DoorCloseComplete(void) {
    return g_blackboard.door.is_closed;
}

bool FSM_CalibrationComplete(void) {
    return g_blackboard.motor.is_calibrated;
}

bool FSM_HasEmergency(void) {
    return g_blackboard.emergency_stop;
}

bool FSM_HasFault(void) {
    return !g_blackboard.motor_safety_ok || !g_blackboard.door_safety_ok;
}

bool FSM_CanRecoverFromFault(void) {
    return !g_blackboard.emergency_stop && 
           g_blackboard.motor_safety_ok && 
           g_blackboard.door_safety_ok;
}

// 动作函数实现
void FSM_StartMovingUp(void) {
    if (Blackboard_IsSafeToMove()) {
        int32_t target_pos = Blackboard_FloorToPosition(g_blackboard.target_floor);
        MotorAdv_MoveToPosition(&motor_adv, target_pos);
        g_blackboard.motor.is_moving = true;
        g_blackboard.motor_running = true;
        g_blackboard.dir = DIR_UP;
        Blackboard_SetState(ELEVATOR_MOVING_UP);
    }
}

void FSM_StartMovingDown(void) {
    if (Blackboard_IsSafeToMove()) {
        int32_t target_pos = Blackboard_FloorToPosition(g_blackboard.target_floor);
        MotorAdv_MoveToPosition(&motor_adv, target_pos);
        g_blackboard.motor.is_moving = true;
        g_blackboard.motor_running = true;
        g_blackboard.dir = DIR_DOWN;
        Blackboard_SetState(ELEVATOR_MOVING_DOWN);
    }
}

void FSM_StopAtFloor(void) {
    Motor_Stop(&motor);
    g_blackboard.motor.is_moving = false;
    Blackboard_PushEvent(EVENT_POSITION_REACHED, g_blackboard.current_floor);
}

void FSM_OpenDoor(void) {
    // 发送开门命令到Slave MCU
    RS485_SendDoorCommand(0x01);  // DOOR_OPEN
    g_blackboard.door.is_moving = true;
}

void FSM_CloseDoor(void) {
    // 发送关门命令到Slave MCU  
    RS485_SendDoorCommand(0x02);  // DOOR_CLOSE
    g_blackboard.door.is_moving = true;
}

void FSM_EmergencyStop(void) {
    Motor_Stop(&motor);
    g_blackboard.motor.is_moving = false;
    Blackboard_SetState(ELEVATOR_EMERGENCY_STOP);
}

void FSM_StartCalibration(void) {
    // 开始校准流程：向下移动寻找底层
    Motor_SetSpeed(&motor, 500);  // 慢速
    Motor_MoveSteps(&motor, -100000);  // 向下移动足够的距离
}

void FSM_ClearFault(void) {
    g_blackboard.motor_safety_ok = true;
    g_blackboard.door_safety_ok = true;
    g_blackboard.emergency_stop = false;
}

// 辅助函数实现
bool FSM_IsAtTargetFloor(void) {
    return g_blackboard.current_floor == g_blackboard.target_floor &&
           !g_blackboard.motor.is_moving;
}

bool FSM_HasCallsAbove(void) {
    uint8_t current = g_blackboard.current_floor;
    for (int i = current + 1; i < MAX_FLOORS; i++) {
        if (g_blackboard.floors[i].call_up || 
            g_blackboard.floors[i].call_down || 
            g_blackboard.floors[i].floor_request) {
            return true;
        }
    }
    return false;
}

bool FSM_HasCallsBelow(void) {
    uint8_t current = g_blackboard.current_floor;
    for (int i = 0; i < current; i++) {
        if (g_blackboard.floors[i].call_up || 
            g_blackboard.floors[i].call_down || 
            g_blackboard.floors[i].floor_request) {
            return true;
        }
    }
    return false;
}

void FSM_ClearFloorCalls(uint8_t floor) {
    if (floor < MAX_FLOORS) {
        Blackboard_ClearPendingCall(floor + 1);  // 转换为1-3编号
    }
}

// 新增辅助函数
static void update_target_floor(void) {
    uint8_t new_target = Blackboard_GetNextTargetFloor();
    if (new_target != g_blackboard.current_floor) {
        g_blackboard.target_floor = new_target;
    }
}

static void check_door_timeout(void) {
    if (g_blackboard.door.is_open && 
        HAL_GetTick() - g_blackboard.door_open_time > g_blackboard.door_timeout) {
        Blackboard_PushEvent(EVENT_CLOSE_DOOR, g_blackboard.current_floor);
    }
}

static void handle_error(void) {
    g_blackboard.motor_running = false;
    Motor_Stop(&motor);
    // 发送错误通知
    RS485_SendCommand(CMD_ERROR, (uint8_t*)&g_blackboard.error_code, 2);
}