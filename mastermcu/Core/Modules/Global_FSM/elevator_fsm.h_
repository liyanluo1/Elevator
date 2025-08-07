#ifndef __ELEVATOR_FSM_H
#define __ELEVATOR_FSM_H

#include "blackboard.h"
#include <stdbool.h>

// FSM状态处理函数指针类型
typedef void (*StateHandler_t)(void);

// FSM转换条件函数指针类型
typedef bool (*TransitionCondition_t)(void);

// 状态转换结构体
typedef struct {
    ElevatorState_t from_state;
    ElevatorState_t to_state;
    TransitionCondition_t condition;
    EventType_t trigger_event;  // 触发事件（可选）
} StateTransition_t;

// FSM初始化
void FSM_Init(void);

// FSM主循环（应在主循环中调用）
void FSM_Process(void);

// 事件处理
void FSM_HandleEvent(Event_t* event);

// 状态处理函数声明
void FSM_StateIdle(void);
void FSM_StateMovingUp(void);
void FSM_StateMovingDown(void);
void FSM_StateDoorOpening(void);
void FSM_StateDoorOpen(void);
void FSM_StateDoorClosing(void);
void FSM_StateDoorClosed(void);
void FSM_StateEmergencyStop(void);
void FSM_StateCalibrating(void);
void FSM_StateFault(void);

// 转换条件函数声明
bool FSM_CanStartMoving(void);
bool FSM_ShouldStopAtFloor(void);
bool FSM_DoorOpenComplete(void);
bool FSM_DoorCloseComplete(void);
bool FSM_CalibrationComplete(void);
bool FSM_HasEmergency(void);
bool FSM_HasFault(void);
bool FSM_CanRecoverFromFault(void);

// 动作函数声明
void FSM_StartMovingUp(void);
void FSM_StartMovingDown(void);
void FSM_StopAtFloor(void);
void FSM_OpenDoor(void);
void FSM_CloseDoor(void);
void FSM_EmergencyStop(void);
void FSM_StartCalibration(void);
void FSM_ClearFault(void);

// 辅助函数
bool FSM_IsAtTargetFloor(void);
bool FSM_HasCallsAbove(void);
bool FSM_HasCallsBelow(void);
void FSM_ClearFloorCalls(uint8_t floor);
void FSM_UpdateDirection(void);

#endif /* __ELEVATOR_FSM_H */