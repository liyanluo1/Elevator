# CANopen DS402 步进电机控制模块说明

## 概述
本模块实现了基于CANopen DS402标准的步进电机控制，支持位置模式(PP)、速度模式(PV)、回原点模式(HM)等多种工作模式。

## 文件结构
- `canopen_stepper.h` - 头文件，定义了所有结构体和接口函数
- `canopen_stepper.c` - 实现文件，包含核心控制逻辑
- `canopen_stepper_example.c` - 使用示例

## 主要功能

### 1. DS402状态机管理
支持标准DS402状态机的所有状态：
- Not Ready to Switch On
- Switch On Disabled
- Ready to Switch On
- Switched On
- Operation Enabled
- Quick Stop Active
- Fault Reaction Active
- Fault

### 2. 工作模式
- **PP模式（轮廓位置模式）**：点到点位置控制
- **PV模式（轮廓速度模式）**：速度控制
- **HM模式（回原点模式）**：自动回原点
- **PT模式（轮廓力矩模式）**：力矩控制（可选）

### 3. SDO通信
实现了完整的SDO读写功能，支持1/2/4字节数据传输。

## 使用步骤

### 1. 初始化
```c
// 创建步进电机实例
CANopen_Stepper_t stepper;

// 初始化（节点ID=1）
CANopen_Stepper_Init(&stepper, 1);

// 设置基本参数
CANopen_SetSubdivision(&stepper, 4000);    // 4000脉冲/圈
CANopen_SetMotorCurrent(&stepper, 3000);   // 3A电流

// 故障复位
CANopen_FaultReset(&stepper);

// 使能运行
CANopen_EnableOperation(&stepper);
```

### 2. 位置控制模式
```c
// 切换到位置模式
CANopen_SetOperationMode(&stepper, MODE_PROFILE_POSITION);

// 设置运动参数
CANopen_SetProfileVelocity(&stepper, 8000);         // 8000脉冲/秒
CANopen_SetProfileAcceleration(&stepper, 2000, 2000); // 加减速度

// 绝对位置移动
CANopen_SetTargetPosition(&stepper, 30000, true);
CANopen_StartPositionMove(&stepper);

// 相对位置移动
CANopen_SetTargetPosition(&stepper, 10000, false);
CANopen_StartPositionMove(&stepper);

// 等待到达
while (!CANopen_IsTargetReached(&stepper)) {
    CANopen_Task(&stepper);
    HAL_Delay(10);
}
```

### 3. 速度控制模式
```c
// 切换到速度模式
CANopen_SetOperationMode(&stepper, MODE_PROFILE_VELOCITY);

// 设置参数
CANopen_SetProfileAcceleration(&stepper, 2000, 2000);
CANopen_SetTargetVelocity(&stepper, 8000);  // 正向8000脉冲/秒

// 启动
CANopen_StartVelocityMode(&stepper);

// 改变速度
CANopen_SetTargetVelocity(&stepper, -1000); // 反向1000脉冲/秒

// 停止
CANopen_HaltVelocityMode(&stepper);
```

### 4. 回原点模式
```c
// 切换到回原模式
CANopen_SetOperationMode(&stepper, MODE_HOMING);

// 设置回原参数
CANopen_SetHomingMethod(&stepper, 24);         // 方法24
CANopen_SetHomingSpeeds(&stepper, 4000, 2000); // 快慢速度
CANopen_SetHomingAcceleration(&stepper, 1000);

// 启动回原
CANopen_StartHoming(&stepper);

// 等待完成
while (!CANopen_IsHomingComplete(&stepper)) {
    CANopen_Task(&stepper);
    HAL_Delay(10);
}
```

### 5. CAN消息处理
在CAN接收中断中调用：
```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        CANopen_ProcessRxMessage(&stepper, rx_header.StdId, rx_data);
    }
}
```

### 6. 周期性任务
在主循环中定期调用：
```c
while (1) {
    CANopen_Task(&stepper);
    // 其他任务...
    HAL_Delay(10);
}
```

## 控制字说明（6040h）

| Bit | 名称 | 说明 |
|-----|------|------|
| 0 | Switch On | 开机 |
| 1 | Enable Voltage | 使能电压 |
| 2 | Quick Stop | 快速停止（0有效） |
| 3 | Enable Operation | 使能运行 |
| 4 | New Setpoint | 新位置触发（PP模式） |
| 5 | Change Set Immediately | 立即生效（PP模式） |
| 6 | Abs/Rel | 0=绝对位置，1=相对位置 |
| 7 | Fault Reset | 故障复位 |
| 8 | Halt | 暂停 |

## 状态字说明（6041h）

| Bit | 名称 | 说明 |
|-----|------|------|
| 0 | Ready to Switch On | 准备开机 |
| 1 | Switched On | 已开机 |
| 2 | Operation Enabled | 运行使能 |
| 3 | Fault | 故障 |
| 4 | Voltage Enabled | 电压使能 |
| 5 | Quick Stop | 快速停止（0有效） |
| 6 | Switch On Disabled | 开机禁用 |
| 7 | Warning | 警告 |
| 9 | Remote | 远程控制 |
| 10 | Target Reached | 目标到达 |
| 11 | Internal Limit | 内部限位 |
| 12 | Op Mode Specific | 模式特定位1 |
| 13 | Op Mode Specific | 模式特定位2 |

## 注意事项

1. **初始化顺序**：必须按照DS402状态机要求，先复位故障，再按顺序切换到运行使能状态。

2. **模式切换**：切换工作模式前，建议先禁用运行，切换后再重新使能。

3. **SDO通信**：每次SDO通信后建议等待10-50ms，确保驱动器处理完成。

4. **错误处理**：定期检查状态字的故障位，及时处理错误状态。

5. **心跳监控**：通过`is_connected`标志监控通信状态。

## 故障处理

当检测到故障时：
1. 读取错误寄存器（可选）
2. 执行故障复位：`CANopen_FaultReset(&stepper)`
3. 等待100ms
4. 重新初始化状态机

## 性能优化建议

1. 使用PDO代替SDO进行频繁的数据交换
2. 合理设置加减速参数，避免电机失步
3. 根据实际负载调整电流设置
4. 使用适当的细分数平衡精度和速度