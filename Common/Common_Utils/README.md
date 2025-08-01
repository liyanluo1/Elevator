# Common Utils Module - 通用工具函数模块

## 概述
此模块提供Master MCU和Slave MCU共享的通用工具函数，包括去抖动、环形缓冲区、GPIO辅助函数等。

## 文件说明

### 1. debounce.h/c - 去抖动器
用于按钮、开关等输入信号的去抖动处理。

**主要功能：**
- 单个输入去抖动
- 批量输入去抖动（最多16个）
- 可配置的去抖时间
- 状态变化检测

**使用示例：**
```c
// 单个按钮
Debouncer_t button_debouncer;
Debouncer_Init(&button_debouncer, 50); // 50ms去抖

// 在定时循环中
bool raw_state = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
Debouncer_Update(&button_debouncer, raw_state, HAL_GetTick());

if (Debouncer_HasChanged(&button_debouncer) && Debouncer_GetState(&button_debouncer)) {
    // 按钮按下
    Debouncer_ClearChange(&button_debouncer);
}
```

### 2. ring_buffer.h/c - 环形缓冲区
用于UART、CAN等通信数据缓存。

**主要功能：**
- 高效的环形缓冲区实现
- 单字节/多字节读写
- 空/满状态检查
- 静态分配宏

**使用示例：**
```c
// 静态分配
RING_BUFFER_DECLARE(uart_rx_buffer, 256);

// 或动态分配
uint8_t buffer[256];
RingBuffer_t rb;
RingBuffer_Init(&rb, buffer, sizeof(buffer));

// 写入数据
RingBuffer_Put(&rb, data);
RingBuffer_Write(&rb, array, length);

// 读取数据
uint8_t byte;
if (RingBuffer_Get(&rb, &byte)) {
    // 处理数据
}
```

### 3. gpio_utils.h/c - GPIO辅助函数
简化GPIO操作的辅助函数。

**主要功能：**
- 简化的GPIO初始化
- 批量GPIO配置
- 便捷的读写操作
- LED/按钮控制宏

**使用示例：**
```c
// 快速初始化
GPIO_Init_Pin(GPIOA, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);

// 批量初始化
GPIO_Config_t configs[] = {
    {GPIOA, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
    {GPIOA, GPIO_PIN_1, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW}
};
GPIO_Init_Pins(configs, 2);

// LED控制
LED_ON(GPIOA, GPIO_PIN_0);
LED_TOGGLE(GPIOA, GPIO_PIN_0);

// 按钮检测
if (BUTTON_PRESSED(GPIOB, GPIO_PIN_0)) {
    // 按钮按下
}
```

## 集成步骤

### 1. 复制到项目
将整个Common_Utils文件夹复制到：
- `mastermcu/Core/Modules/Common_Utils/`
- `slavemcu/Core/Modules/Common_Utils/`

### 2. 添加到编译路径
在IDE中添加源文件：
- `Common_Utils/debounce.c`
- `Common_Utils/ring_buffer.c`
- `Common_Utils/gpio_utils.c`

### 3. 包含头文件
```c
#include "Common_Utils/debounce.h"
#include "Common_Utils/ring_buffer.h"
#include "Common_Utils/gpio_utils.h"
```

### 4. 根据MCU调整
对于STM32F1（slavemcu），可能需要调整gpio_utils.h中的HAL库包含：
```c
#include "stm32f1xx_hal.h"  // 对于F1系列
```

## 优势
- 减少重复代码
- 经过验证的实现
- 提高开发效率
- 统一的接口设计

## 注意事项
- GPIO工具需要根据具体MCU系列调整
- 环形缓冲区不是线程安全的，在中断中使用需要注意
- 去抖动器依赖系统时钟（HAL_GetTick）