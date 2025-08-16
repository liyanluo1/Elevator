#include "elevator_control.h"
#include "stm32f4xx_hal.h"

/* 全局电梯实例 */
ElevatorControl_t g_elevator;

/* 私有函数 */
static void ElevatorControl_UpdateState(void);
static void ElevatorControl_SetState(ElevatorState_t new_state);

/* 初始化电梯控制 */
void ElevatorControl_Init(void) {
    /* 初始化底层电机 */
    StepperMotor_Init(&g_elevator.motor);
    
    /* 初始化楼层信息 */
    g_elevator.current_floor = 1;
    g_elevator.target_floor = 1;
    
    /* 初始化状态 */
    g_elevator.state = ELEVATOR_STATE_IDLE;
    g_elevator.prev_state = ELEVATOR_STATE_IDLE;
    g_elevator.state_enter_time = HAL_GetTick();
    
    /* 初始化楼层位置（理想位置） */
    for (int i = 0; i <= ELEVATOR_MAX_FLOOR; i++) {
        g_elevator.floor_positions[i] = i * ELEVATOR_STEPS_PER_FLOOR;
    }
    g_elevator.is_calibrated = false;
    
    /* 使能电机 */
    StepperMotor_Enable(&g_elevator.motor);
}

/* 重置电梯 */
void ElevatorControl_Reset(void) {
    StepperMotor_Reset(&g_elevator.motor);
    ElevatorControl_Init();
}

/* 移动到指定楼层 */
void ElevatorControl_MoveToFloor(int8_t floor) {
    /* 检查楼层范围 */
    if (floor < ELEVATOR_MIN_FLOOR || floor > ELEVATOR_MAX_FLOOR) {
        return;
    }
    
    /* 如果已在目标楼层，不执行 */
    if (floor == g_elevator.current_floor && g_elevator.state == ELEVATOR_STATE_IDLE) {
        return;
    }
    
    /* 设置目标楼层 */
    g_elevator.target_floor = floor;
    
    /* 计算目标位置 */
    int32_t target_position = g_elevator.floor_positions[floor];
    
    /* 启动电机移动 */
    StepperMotor_MoveAbsolute(&g_elevator.motor, target_position);
    
    /* 更新状态 */
    ElevatorControl_SetState(ELEVATOR_STATE_MOVING);
}

/* 相对移动楼层数 */
void ElevatorControl_MoveFloors(int8_t floors) {
    int8_t target_floor = g_elevator.current_floor + floors;
    
    /* 限制在有效范围内 */
    if (target_floor < ELEVATOR_MIN_FLOOR) {
        target_floor = ELEVATOR_MIN_FLOOR;
    } else if (target_floor > ELEVATOR_MAX_FLOOR) {
        target_floor = ELEVATOR_MAX_FLOOR;
    }
    
    /* 如果没有移动，直接返回 */
    if (target_floor == g_elevator.current_floor) {
        return;
    }
    
    /* 设置目标楼层 */
    g_elevator.target_floor = target_floor;
    
    /* 计算相对步数 */
    int32_t steps = floors * ELEVATOR_STEPS_PER_FLOOR;
    
    /* 启动相对移动 */
    StepperMotor_MoveRelative(&g_elevator.motor, steps);
    
    /* 更新状态 */
    ElevatorControl_SetState(ELEVATOR_STATE_MOVING);
}

/* 停止电梯 */
void ElevatorControl_Stop(void) {
    StepperMotor_Stop(&g_elevator.motor);
    ElevatorControl_SetState(ELEVATOR_STATE_IDLE);
}

/* 校准当前楼层 */
void ElevatorControl_Calibrate(int8_t floor) {
    if (floor < ELEVATOR_MIN_FLOOR || floor > ELEVATOR_MAX_FLOOR) {
        return;
    }
    
    /* 记录当前位置为该楼层位置 */
    int32_t current_pos = StepperMotor_GetPosition(&g_elevator.motor);
    g_elevator.floor_positions[floor] = current_pos;
    g_elevator.current_floor = floor;
    g_elevator.is_calibrated = true;
}

/* 设置当前楼层 */
void ElevatorControl_SetCurrentFloor(int8_t floor) {
    if (floor >= ELEVATOR_MIN_FLOOR && floor <= ELEVATOR_MAX_FLOOR) {
        g_elevator.current_floor = floor;
    }
}

/* 获取当前状态 */
ElevatorState_t ElevatorControl_GetState(void) {
    return g_elevator.state;
}

/* 获取当前楼层 */
int8_t ElevatorControl_GetCurrentFloor(void) {
    return g_elevator.current_floor;
}

/* 获取目标楼层 */
int8_t ElevatorControl_GetTargetFloor(void) {
    return g_elevator.target_floor;
}

/* 是否在移动 */
bool ElevatorControl_IsMoving(void) {
    return g_elevator.state == ELEVATOR_STATE_MOVING;
}

/* 是否空闲 */
bool ElevatorControl_IsIdle(void) {
    return g_elevator.state == ELEVATOR_STATE_IDLE;
}

/* 主循环更新 */
void ElevatorControl_Update(void) {
    /* 更新底层电机 */
    StepperMotor_Update(&g_elevator.motor);
    
    /* 更新状态机 */
    ElevatorControl_UpdateState();
}

/* 更新状态机 */
static void ElevatorControl_UpdateState(void) {
    uint32_t current_time = HAL_GetTick();
    
    switch (g_elevator.state) {
        case ELEVATOR_STATE_IDLE:
            /* 空闲状态，等待命令 */
            break;
            
        case ELEVATOR_STATE_MOVING:
            /* 检查是否到达目标 */
            if (!StepperMotor_IsMoving(&g_elevator.motor)) {
                /* 更新当前楼层 */
                g_elevator.current_floor = g_elevator.target_floor;
                /* 直接转为空闲状态 */
                ElevatorControl_SetState(ELEVATOR_STATE_IDLE);
            }
            break;
    }
}

/* 设置新状态 */
static void ElevatorControl_SetState(ElevatorState_t new_state) {
    if (new_state != g_elevator.state) {
        g_elevator.prev_state = g_elevator.state;
        g_elevator.state = new_state;
        g_elevator.state_enter_time = HAL_GetTick();
    }
}

/* 获取状态名称 */
const char* ElevatorControl_GetStateName(ElevatorState_t state) {
    switch (state) {
        case ELEVATOR_STATE_IDLE:    return "IDLE";
        case ELEVATOR_STATE_MOVING:  return "MOVING";
        default:                     return "UNKNOWN";
    }
}