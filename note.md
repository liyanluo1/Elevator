
# Event Queue -> 容器 (环形缓冲区实现)

## Event 结构
1. **EventType_t** -> 事件类型
   - EVENT_BUTTON_PRESSED (按钮按下)
   - EVENT_FLOOR_REACHED (到达楼层)
   - EVENT_DOOR_OPENED/CLOSED (门开/关)
   - EVENT_CALIBRATION_DONE (校准完成)
   - EVENT_ERROR_OCCURRED (错误发生)
   
2. **timestamp** -> 事件发生时间 (HAL_GetTick())

3. **floor** -> 事件发生楼层 (0-2表示1-3楼)
   - 楼层切换事件流: ELEVATOR_MOVING_UP -> EVENT_POSITION_REACHED -> FSM_ShouldStopAtFloor() -> ELEVATOR_DOOR_OPENING
   
4. **data[4]** -> 额外数据
   - 按钮类型 (BUTTON_CALL_UP/DOWN/FLOOR)
   - 错误代码
   - 位置偏移量

## 生产者 (Event Producers) - 共计38个

### Master MCU (22个生产者):

1. **按钮处理** (button_handler.c)
   - EVENT_OPEN_DOOR - 开门按钮按下
   
2. **电机状态机** (motor_fsm.c)
   - EVENT_STOP_MOVING - 电机停止
   - EVENT_POSITION_ADJUST - 位置调整
   - EVENT_ERROR - 电机错误
   
3. **RS485通信** (rs485_master.c)
   - EVENT_COMM_TIMEOUT - 通信超时
   - EVENT_SYNC_TIMEOUT - 同步超时
   - EVENT_ERROR - RS485错误
   - EVENT_DOOR_OPENED - 门已打开
   - EVENT_DOOR_CLOSED - 门已关闭
   - EVENT_SENSOR_TRIGGERED - 传感器触发
   
4. **电梯FSM** (elevator_fsm.c) - 最多事件
   - EVENT_OPEN_DOOR - 决策开门
   - EVENT_START_MOVING - 开始移动
   - EVENT_OPEN_DOOR - 到达目标开门
   - EVENT_MOTOR_FAULT - 电机故障
   - EVENT_CALIBRATION_DONE - 校准完成
   - EVENT_POSITION_REACHED - 到达位置
   - EVENT_CLOSE_DOOR - 自动关门
   
5. **校准模块** (calibration.c)
   - EVENT_CALIBRATION_DONE - 校准结束
   
6. **黑板内部** (blackboard.c)
   - EVENT_TARGET_UPDATED - 目标楼层更新
   
7. **主应用** (main.c)
   - EVENT_SENSOR_TRIGGERED - 传感器输入处理

### Slave MCU (16个生产者):

1. **光电传感器** (photo_sensor.c)
   - EVENT_POSITION_ADJUST - 位置调整
   - EVENT_ERROR - 传感器超时
   
2. **RS485从机适配器** (rs485_slave_adapter.c)
   - EVENT_SYNC_TIMEOUT - 同步超时
   - EVENT_ERROR - RS485错误
   - EVENT_OPEN_DOOR - 收到开门命令
   - EVENT_CLOSE_DOOR - 收到关门命令
   - EVENT_ERROR - 错误处理
   
3. **键盘模块** (keyboard.c)
   - EVENT_TARGET_UPDATED - 楼层选择
   - EVENT_OPEN_DOOR - 开门键
   - EVENT_CLOSE_DOOR - 关门键
   - EVENT_ERROR - 键盘错误
   
4. **本地黑板内部** (local_blackboard.c)
   - EVENT_DOOR_OPENED - 门开启通知
   - EVENT_DOOR_CLOSED - 门关闭通知
   - EVENT_FLOOR_REACHED - 到达楼层
   - EVENT_TARGET_UPDATED - 目标更新
   - EVENT_SENSOR_TRIGGERED - 传感器触发
   - EVENT_KEYBOARD_INPUT - 键盘输入
   
5. **主应用** (main.c)
   - EVENT_OPEN_DOOR - 开门超时
   - EVENT_CLOSE_DOOR - 关门超时

## 间接事件生成

### 1. 定时器基事件:
- **超时检测**: RS485通信超时自动生成EVENT_COMM_TIMEOUT
- **校准超时**: 传感器校准失败生成EVENT_ERROR
- **门操作超时**: 开/关门超时生成对应事件

### 2. 状态机转换:
- **电梯FSM**: 状态变化触发级联事件
- **电机FSM**: 位置更新生成调整事件
- **传感器FSM**: 检测状态变化创建位置事件

### 3. 中断事件源:
- **GPIO EXTI** (Slave): PhotoSensor_ISR() 触发传感器处理
- **UART RX/TX** (Both): 通信完成事件
- **CAN RX** (Master): 电机响应处理
- **SysTick**: 系统计时（未直接生成事件）


## 消费者 (Event Consumers) - 各1个主循环

### Master MCU - 单一消费者:
**电梯FSM** (elevator_fsm.c):
```c
void FSM_Process(void) {
    Event_t event;
    // 从队列取出所有事件并处理
    while (Blackboard_PopEvent(&event)) {
        FSM_HandleEvent(&event);  // 处理每个事件
    }
    // 继续状态机处理
    FSM_StateMachine();
}
```

### Slave MCU - 单一消费者:
**主应用循环** (main.c):
```c
static void ProcessLocalEvents(void) {
    LocalEvent_t event;
    while (LocalBlackboard_PopEvent(&event)) {
        switch (event.type) {
            case EVENT_OPEN_DOOR:
                Servo_SetDoorOpen();
                g_local_bb.servo_state = SERVO_OPENING;
                break;
                
            case EVENT_CLOSE_DOOR:
                Servo_SetDoorClose();
                g_local_bb.servo_state = SERVO_CLOSING;
                break;
                
            case EVENT_FLOOR_REACHED:
                // 检查是否需要开门
                if (event.data == g_local_bb.target_floor) {
                    LocalBlackboard_PushEvent(EVENT_OPEN_DOOR, 0);
                }
                break;
                
            case EVENT_POSITION_ADJUST:
                // 发送位置调整到Master
                RS485_SendPositionAdjust(event.data);
                break;
                
            // ...其他事件处理
        }
    }
}
```

## 事件流示例: 上行到达楼层并开门

1. **电机到位**: Motor_Task() 检测到达目标位置
2. **生成事件**: Blackboard_PushEvent(EVENT_POSITION_REACHED, current_floor)
3. **FSM处理**: FSM_HandleEvent() 接收并判断是否停靠
4. **停靠决策**: FSM_ShouldStopAtFloor() 检查:
   - 是否到达目标楼层
   - 是否有该楼层的上行呼叫
   - 是否有轿厘内该楼层请求
5. **状态转换**: ELEVATOR_MOVING_UP -> ELEVATOR_DOOR_OPENING
6. **发送开门命令**: 通过RS485发送到Slave
7. **Slave执行**: 控制伺服电机开门
8. **反馈确认**: Slave返回EVENT_DOOR_OPENED
9. **最终状态**: ELEVATOR_DOOR_OPEN (等待3秒)
