#ifndef __KEYBOARD_H
#define __KEYBOARD_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// 键盘GPIO配置 - 4x4 Matrix Keypad
// Row pin (input with pullup) - 只使用r4行
#define KEYBOARD_ROW_PORT       GPIOA
#define KEYBOARD_ROW_PIN        GPIO_PIN_11    // r4 (for S13-S16)

// Column pins (output) - 实际物理连接
#define KEYBOARD_COL_PORT       GPIOA
#define KEYBOARD_COL1_PIN       GPIO_PIN_8     // PA8 -> S15 -> Floor 2
#define KEYBOARD_COL2_PIN       GPIO_PIN_12    // PA12 -> S13 -> Not used
#define KEYBOARD_COL3_PIN       GPIO_PIN_5     // PA5 -> S14 -> Floor 3
#define KEYBOARD_COL4_PIN       GPIO_PIN_4     // PA4 -> S16 -> Floor 1

// 键盘参数
#define KEYBOARD_DEBOUNCE_TIME  5       // 去抖时间（ms） - 降低到5ms
#define KEYBOARD_SCAN_PERIOD    5       // 扫描周期（ms） - 降低到5ms
#define KEYBOARD_BUFFER_SIZE    8       // 键值缓冲区大小
#define KEYBOARD_RELEASE_TIME   10      // 按键释放确认时间（ms） - 降低到10ms

// 键值定义 - Elevator car call buttons
#define KEY_NONE                0
#define KEY_S13                 13      // Not used in elevator logic
#define KEY_S14                 14      // Floor 3 button
#define KEY_S15                 15      // Floor 2 button
#define KEY_S16                 16      // Floor 1 button

// 键盘状态结构体
typedef struct {
    // 扫描相关
    uint8_t current_key;            // 当前按键
    uint8_t last_key;               // 上次按键
    uint32_t last_scan_time;        // 上次扫描时间
    uint32_t key_press_time;        // 按键按下时间
    
    // 去抖相关
    uint8_t debounce_count;         // 去抖计数
    bool key_stable;                // 按键稳定标志
    
    // 缓冲区
    uint8_t key_buffer[KEYBOARD_BUFFER_SIZE];  // 键值缓冲区
    uint8_t buffer_head;            // 缓冲区头
    uint8_t buffer_tail;            // 缓冲区尾
    uint8_t buffer_count;           // 缓冲区计数
    
    // 中断标志
    volatile bool interrupt_flag;   // 中断触发标志
    volatile uint32_t interrupt_time; // 中断触发时间
    
    // 统计
    uint32_t total_key_presses;     // 总按键次数
    uint32_t invalid_key_count;     // 无效按键计数
    
} Keyboard_t;

// 全局键盘实例
extern Keyboard_t g_keyboard;

// 初始化和处理函数
void Keyboard_Init(void);
void Keyboard_Handler(void);

// 键盘扫描函数
uint8_t Keyboard_Scan(void);
uint8_t Keyboard_ScanInterrupt(void);
uint8_t Keyboard_ReadMatrix(void);
bool Keyboard_IsKeyPressed(uint8_t key);

// 缓冲区操作
bool Keyboard_PushKey(uint8_t key);
bool Keyboard_PopKey(uint8_t* key);
bool Keyboard_HasKey(void);
void Keyboard_ClearBuffer(void);

// 功能处理
void Keyboard_ProcessKey(uint8_t key);

// GPIO配置
void Keyboard_GPIO_Init(void);

// 中断回调
void Keyboard_IRQHandler(void);

// 调试函数
void Keyboard_PrintStatus(void);

#endif /* __KEYBOARD_H */
