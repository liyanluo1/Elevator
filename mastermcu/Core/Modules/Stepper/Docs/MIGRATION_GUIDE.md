# CANopen步进电机控制迁移指南

## 概述
本指南帮助您从旧的专有电机控制协议迁移到标准的CANopen DS402协议。

## 文件变更说明

### 1. 需要保留的文件
- `canopen_stepper.h/c` - 新的CANopen DS402实现
- `canopen_stepper_example.c` - 使用示例
- `motor_control_canopen_adapter.c` - 向后兼容适配器
- `motor_fsm.h/c` - 需要修改以适配DS402状态机

### 2. 已废弃的文件
- `motor_control.h/c` - 旧的专有协议（使用适配器保持兼容）
- `motor_advanced.h/c` - 高级功能已集成到CANopen

### 3. CAN模块
- **必须保留** `CAN/can.h` 和 `CAN/can.c`
- CANopen依赖底层CAN通信功能

## 代码迁移步骤

### 方式1：使用适配器（推荐，无需修改现有代码）
```c
// 在main.c中添加
#define USE_CANOPEN_ADAPTER

// 现有代码无需修改，自动使用CANopen
Motor_Init(&motor, 1);
Motor_MoveTo(&motor, 10000);
```

### 方式2：直接使用CANopen（推荐用于新代码）
```c
#include "canopen_stepper.h"

// 初始化
CANopen_Stepper_t stepper;
CANopen_Stepper_Init(&stepper, 1);
CANopen_SetSubdivision(&stepper, 4000);
CANopen_FaultReset(&stepper);
CANopen_EnableOperation(&stepper);

// 位置控制
CANopen_SetOperationMode(&stepper, MODE_PROFILE_POSITION);
CANopen_SetTargetPosition(&stepper, 10000, true);
CANopen_StartPositionMove(&stepper);
```

## 功能对比

| 旧API | CANopen API | 说明 |
|-------|-------------|------|
| Motor_Init() | CANopen_Stepper_Init() | 初始化 |
| Motor_MoveTo() | CANopen_SetTargetPosition() + StartPositionMove() | 绝对定位 |
| Motor_MoveSteps() | CANopen_SetTargetPosition(relative=false) | 相对定位 |
| Motor_SetSpeed() | CANopen_SetProfileVelocity() | 设置速度 |
| Motor_Stop() | CANopen_QuickStop() | 急停 |
| Motor_SetZero() | CANopen_SetHomingMethod(35) + StartHoming() | 设置零点 |

## 新增功能

### 1. 速度模式
```c
CANopen_SetOperationMode(&stepper, MODE_PROFILE_VELOCITY);
CANopen_SetTargetVelocity(&stepper, 8000);
CANopen_StartVelocityMode(&stepper);
```

### 2. 回原点模式
```c
CANopen_SetOperationMode(&stepper, MODE_HOMING);
CANopen_SetHomingMethod(&stepper, 24);
CANopen_StartHoming(&stepper);
```

### 3. DS402标准状态机
- 完整的故障处理
- 标准化的状态转换
- 更好的错误恢复

## 编译配置

在项目设置中：
1. 如果使用适配器，定义 `USE_CANOPEN_ADAPTER`
2. 确保包含路径包含 `Core/Modules/Stepper`
3. 链接所有必要的文件

## 注意事项

1. CANopen需要更多的初始化步骤（故障复位、使能等）
2. 状态机更严格，必须按正确顺序操作
3. SDO通信需要等待响应（建议10-50ms延时）
4. 心跳监控确保通信可靠性

## 故障排除

### 问题：电机不响应
- 检查节点ID是否正确
- 确认已执行故障复位
- 验证状态机在"Operation Enabled"状态

### 问题：位置不准确
- 检查细分设置是否匹配
- 确认加减速参数合理
- 验证是绝对还是相对定位

## 联系支持
如有问题，请查看：
- `canopen_stepper_example.c` - 完整示例
- `CANopen步进电机控制说明.md` - 详细文档