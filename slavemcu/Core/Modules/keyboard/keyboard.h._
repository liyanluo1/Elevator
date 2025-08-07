#ifndef __KEYBOARD_H
#define __KEYBOARD_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "local_blackboard.h"

// 键盘GPIO配置（4x4矩阵键盘）
#define KEYBOARD_ROW_PORT       GPIOA
#define KEYBOARD_COL_PORT       GPIOB

#define KEYBOARD_ROW1_PIN       GPIO_PIN_0
#define KEYBOARD_ROW2_PIN       GPIO_PIN_1
#define KEYBOARD_ROW3_PIN       GPIO_PIN_2
#define KEYBOARD_ROW4_PIN       GPIO_PIN_3

#define KEYBOARD_COL1_PIN       GPIO_PIN_12
#define KEYBOARD_COL2_PIN       GPIO_PIN_13
#define KEYBOARD_COL3_PIN       GPIO_PIN_14
#define KEYBOARD_COL4_PIN       GPIO_PIN_15

// 键盘参数
#define KEYBOARD_DEBOUNCE_TIME  50      // 去抖时间（ms）
#define KEYBOARD_SCAN_PERIOD    20      // 扫描周期（ms）
#define KEYBOARD_BUFFER_SIZE    8       // 键值缓冲区大小

// 键值定义
#define KEY_NONE                0
#define KEY_1                   1       // 1楼
#define KEY_2                   2       // 2楼
#define KEY_3                   3       // 3楼
#define KEY_OPEN                10      // 开门键
#define KEY_CLOSE               11      // 关门键
#define KEY_EMERGENCY           12      // 紧急按钮

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
uint8_t Keyboard_ReadMatrix(void);
bool Keyboard_IsKeyPressed(uint8_t key);

// 缓冲区操作
bool Keyboard_PushKey(uint8_t key);
bool Keyboard_PopKey(uint8_t* key);
bool Keyboard_HasKey(void);
void Keyboard_ClearBuffer(void);

// 功能处理
void Keyboard_ProcessKey(uint8_t key);
void Keyboard_HandleFloorRequest(uint8_t floor);
void Keyboard_HandleDoorControl(uint8_t cmd);
void Keyboard_HandleEmergency(void);

// GPIO配置
void Keyboard_GPIO_Init(void);
void Keyboard_SetRow(uint8_t row);
uint8_t Keyboard_ReadCol(void);

// 键值映射
uint8_t Keyboard_MapKey(uint8_t row, uint8_t col);

// 调试函数
void Keyboard_PrintStatus(void);

#endif /* __KEYBOARD_H */
