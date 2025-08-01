# RS485通用模块

## 概述
这是一个模块化的RS485通信框架，提供了Master和Slave MCU之间的可靠通信。通过分层设计，实现了核心功能与具体应用的解耦。

## 目录结构
```
Common/RS485/
├── Core/               # 核心功能
│   ├── rs485_base.h    # 基础定义
│   ├── rs485_core.h    # 核心控制
│   └── rs485_core.c    # 核心实现
├── Protocol/           # 协议层
│   ├── rs485_protocol.h # 协议定义
│   └── rs485_protocol.c # 协议实现
└── Utils/              # 工具函数
    ├── rs485_crc.h     # CRC计算
    └── rs485_crc.c     # CRC实现
```

## 集成步骤

### 1. 添加包含路径
在STM32CubeIDE中，需要为两个MCU项目添加Common目录的包含路径：

#### Master MCU:
1. 右击项目 -> Properties
2. C/C++ Build -> Settings -> Tool Settings
3. MCU GCC Compiler -> Include paths
4. 添加路径：
   - `../../Common/RS485/Core`
   - `../../Common/RS485/Protocol`
   - `../../Common/RS485/Utils`

#### Slave MCU:
执行相同步骤

### 2. 添加源文件
将以下文件添加到项目中：
- `Common/RS485/Core/rs485_core.c`
- `Common/RS485/Protocol/rs485_protocol.c`
- `Common/RS485/Utils/rs485_crc.c`

### 3. 使用示例

#### Master端：
```c
#include "rs485_master.h"

// 初始化
RS485_Master_Init(&g_blackboard);

// 主循环
while (1) {
    RS485_Master_Handler();
    // 其他任务...
}
```

#### Slave端：
```c
#include "rs485_slave_adapter.h"

// 初始化
RS485_SlaveAdapter_Init(&g_local_bb);

// 主循环
while (1) {
    RS485_SlaveAdapter_Handler();
    // 其他任务...
}
```

## 架构优势

1. **模块化设计**：核心功能、协议处理、应用适配器相互独立
2. **统一CRC算法**：确保两端使用相同的CRC计算方法
3. **灵活的命令映射**：支持Master和Slave使用不同的命令值
4. **回调机制**：便于扩展和自定义处理
5. **错误处理**：完善的错误检测和恢复机制
6. **统计信息**：提供通信质量监控

## 协议格式

帧格式：
```
[START(0xAA)][LENGTH][CMD][DATA...][CRC_LOW][CRC_HIGH][END(0x55)]
```

## 配置参数

可在初始化时配置：
- 波特率（默认115200）
- 超时时间（默认200ms）
- 重试次数（默认3次）
- 心跳间隔（默认100ms）

## 扩展指南

1. **添加新命令**：在`rs485_protocol.h`中的`RS485_CommandType_t`枚举添加
2. **自定义处理**：在适配器中实现相应的回调函数
3. **修改CRC算法**：在`rs485_crc.c`中修改算法实现

## 注意事项

1. 确保两端使用相同的波特率和协议参数
2. DE引脚控制需要正确配置
3. UART中断回调需要正确连接
4. 注意STM32F1和STM32F4的HAL库差异