# Common Types Module - 通用类型定义模块

## 概述
此模块包含电梯系统中Master MCU和Slave MCU共享的类型定义、常量和宏。

## 文件说明

### elevator_types.h
包含所有共享的类型定义：
- 基本常量（楼层数、超时时间等）
- 枚举类型（方向、状态、错误码等）
- 事件系统定义
- 通信协议定义
- 实用工具宏

## 集成步骤

### 1. 复制到项目
将整个Common_Types文件夹复制到：
- `mastermcu/Core/Modules/Common_Types/`
- `slavemcu/Core/Modules/Common_Types/`

### 2. 更新包含路径
在需要使用的文件中：
```c
#include "Common_Types/elevator_types.h"
```

### 3. 替换重复定义
搜索并替换项目中的重复定义：
- Direction_t枚举
- 楼层常量定义
- 事件类型定义
- 错误码定义

## 使用示例

### 方向枚举
```c
Direction_t current_dir = DIR_UP;
if (current_dir == DIR_IDLE) {
    // 电梯静止
}
```

### 楼层检查
```c
uint8_t target_floor = 2;
if (IS_VALID_FLOOR(target_floor)) {
    // 有效楼层
}
```

### 事件创建
```c
Event_t event = {
    .type = EVENT_BUTTON_PRESSED,
    .floor = FLOOR_2,
    .data = BUTTON_CALL_UP,
    .timestamp = HAL_GetTick()
};
```

### 错误处理
```c
ErrorCode_t error = ERROR_NONE;
if (motor_fault) {
    SET_BIT(error, ERROR_MOTOR_FAULT);
}
```

## 优势
- 消除重复定义
- 确保Master和Slave使用相同的类型
- 集中管理常量，便于调整
- 提供统一的工具宏

## 注意事项
- 修改后需要同时更新Master和Slave的副本
- 保持类型定义的向后兼容性
- 新增定义时考虑两端的需求