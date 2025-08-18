#ifndef __ELEVATOR_FSM_H
#define __ELEVATOR_FSM_H

#include "../Blackboard/blackboard.h"
#include "../RS485/rs485_protocol.h"

/* FSM初始化 */
void FSM_Init(void);

/* FSM主处理函数（在主循环中调用） */
void FSM_Process(void);

/* 事件处理 */
void FSM_HandleButtonUp(uint8_t floor);
void FSM_HandleButtonDown(uint8_t floor);
void FSM_HandleCabinCall(uint8_t floor);
void FSM_HandlePhotoSensor(uint8_t floor);

/* 内部状态处理函数 */
void FSM_StateIdle(void);
void FSM_StateMoving(void);
void FSM_StateDoorOperating(void);
void FSM_StatePreparing(void);

/* 辅助函数 */
bool FSM_IsTimeout(uint32_t start_time, uint32_t timeout_ms);
void FSM_SendDoorCommand(bool open);
void FSM_SendDirectionCommand(Direction_t dir);

#endif /* __ELEVATOR_FSM_H */