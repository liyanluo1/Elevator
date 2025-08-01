# Stepper Module - 步进电机控制模块

## 目录结构

```
Stepper/
├── CANopen/                    # CANopen DS402标准实现（推荐使用）
│   ├── canopen_stepper.h       # CANopen接口定义
│   ├── canopen_stepper.c       # CANopen实现
│   └── canopen_stepper_example.c # 使用示例
├── motor_control_types.h       # 类型定义（向后兼容）
├── Adapter/                    # 适配层
│   └── motor_control_canopen_adapter.c # 向后兼容适配器
├── FSM/                        # 状态机
│   ├── motor_fsm.h            # 电机状态机定义
│   └── motor_fsm.c            # 电机状态机实现
└── Docs/                      # 文档
    ├── CANopen步进电机控制说明.md # CANopen详细说明
    └── MIGRATION_GUIDE.md      # 迁移指南
```

## 快速开始

### 1. 新项目（推荐）
直接使用CANopen实现：
```c
#include "CANopen/canopen_stepper.h"

CANopen_Stepper_t stepper;
CANopen_Stepper_Init(&stepper, 1);
// 详见 CANopen/canopen_stepper_example.c
```

### 2. 现有项目
使用适配器保持兼容：
```c
#define USE_CANOPEN_ADAPTER
#include "motor_control_types.h"
// 原有代码无需修改
```

## 模块说明

### CANopen/ - 标准CANopen DS402实现
- 完全符合DS402标准
- 支持PP(位置)、PV(速度)、HM(回原点)模式
- 内置故障处理和状态机管理
- 支持SDO通信协议

### motor_control_types.h - 类型定义
- 仅包含必要的类型定义
- 用于向后兼容
- 实际功能由Adapter提供

### Adapter/ - 适配层
- 将旧API映射到新的CANopen实现
- 无缝迁移现有代码
- 提供额外的高级功能接口

### FSM/ - 状态机
- 管理电机的运行状态
- 需要修改以适配DS402状态

## 依赖关系
- 依赖 `../CAN/` 模块提供底层CAN通信
- 依赖 `../Global_bb/` 模块进行全局数据交换

## 更多信息
- 详细使用说明：`Docs/CANopen步进电机控制说明.md`
- 迁移指南：`Docs/MIGRATION_GUIDE.md`
- 示例代码：`CANopen/canopen_stepper_example.c`