# 步进电机控制模块（简化版）

## 概述
本模块提供了一个简化的步进电机控制接口，专门用于电梯控制系统。

## 特点
- **3状态状态机**：IDLE、MOVING、REACHED
- **固定参数**：
  - 节点ID：32
  - 速度：10000 p/s（固定，无加减速）
  - 步数/层：17000步
- **只支持相对位置移动**
- **光电传感器位置校正**

## 文件结构
```
Stepper/
├── stepper_simple.h    # 简化控制接口
├── stepper_simple.c    # 控制实现
├── stepper_fsm.h       # 状态机定义
├── stepper_fsm.c       # 状态机实现
└── README.md          # 本文档
```

## 使用方法

### 初始化
```c
Stepper_t stepper;
Stepper_Init(&stepper);
Stepper_Enable(&stepper);
Stepper_SetCurrentFloor(&stepper, 0);  // 设置当前楼层
```

### 楼层控制
```c
// 移动到指定楼层
Stepper_MoveToFloor(&stepper, 3);

// 相对移动
Stepper_MoveRelativeFloors(&stepper, -2);  // 下降2层
```

### 位置同步（光电传感器）
```c
// 当光电传感器检测到楼层时调用
Stepper_SyncPosition(&stepper, current_floor);
```

### 状态查询
```c
bool is_moving = Stepper_IsMoving(&stepper);
int8_t floor = Stepper_GetCurrentFloor(&stepper);
StepperState_t state = Stepper_GetState(&stepper);
```

### 主循环调用
```c
while(1) {
    Stepper_Update(&stepper);  // 更新状态机
    HAL_Delay(10);
}
```

## 状态转换
```
IDLE ──[START_MOVE]──> MOVING ──[TARGET_REACHED]──> REACHED ──[2秒后]──> IDLE
         ^                |
         └────────────────┘
           [新的移动命令]
```

## CANopen通信
- 使用SDO协议
- COB-ID: 0x600 + NodeID (发送), 0x580 + NodeID (接收)
- 操作模式：Profile Position (PP)
- 控制字bit6=1表示相对位置模式

## 注意事项
1. 电机必须先Enable才能移动
2. 每层楼17000步是固定值
3. 光电传感器用于消除累积误差
4. 不支持加减速控制（固定速度）