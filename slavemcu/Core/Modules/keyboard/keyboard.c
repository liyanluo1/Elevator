#include "keyboard.h"
// #include "rs485_slave.h"  // COMMENTED OUT FOR TESTING
#include <stdio.h>
#include <string.h>

// 全局键盘实例
Keyboard_t g_keyboard;

// 初始化键盘模块
void Keyboard_Init(void) {
    // 清零结构体
    memset(&g_keyboard, 0, sizeof(Keyboard_t));
    
    // 初始化变量
    g_keyboard.current_key = KEY_NONE;
    g_keyboard.last_key = KEY_NONE;
    g_keyboard.last_scan_time = HAL_GetTick();
    g_keyboard.key_stable = false;
    g_keyboard.buffer_head = 0;
    g_keyboard.buffer_tail = 0;
    g_keyboard.buffer_count = 0;
    
    // 初始化GPIO
    Keyboard_GPIO_Init();
}

// 主处理函数
void Keyboard_Handler(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 检查是否到达扫描时间
    if (current_time - g_keyboard.last_scan_time >= KEYBOARD_SCAN_PERIOD) {
        g_keyboard.last_scan_time = current_time;
        
        // 扫描键盘
        uint8_t key = Keyboard_Scan();
        
        // 处理按键
        if (key != KEY_NONE && key != g_keyboard.last_key) {
            Keyboard_ProcessKey(key);
        }
        
        g_keyboard.last_key = key;
    }
    
    // Process buffered keys - simplified for S13-S16 testing
    uint8_t buffered_key;
    if (Keyboard_PopKey(&buffered_key)) {
        // Just count the key press for testing
        g_keyboard.total_key_presses++;
    }
}

// 扫描键盘
uint8_t Keyboard_Scan(void) {
    uint8_t key = Keyboard_ReadMatrix();
    
    // 去抖处理
    if (key != KEY_NONE) {
        if (key == g_keyboard.current_key) {
            g_keyboard.debounce_count++;
            if (g_keyboard.debounce_count >= 3) {  // 连续3次相同读数
                g_keyboard.key_stable = true;
                return key;
            }
        } else {
            g_keyboard.current_key = key;
            g_keyboard.debounce_count = 1;
            g_keyboard.key_stable = false;
        }
    } else {
        g_keyboard.current_key = KEY_NONE;
        g_keyboard.debounce_count = 0;
        g_keyboard.key_stable = false;
    }
    
    return KEY_NONE;
}

// Read keyboard matrix - Modified for S13-S16 (r4 row only)
uint8_t Keyboard_ReadMatrix(void) {
    uint8_t key = KEY_NONE;
    
    // Set all columns HIGH first
    HAL_GPIO_WritePin(KEYBOARD_COL_PORT, 
                      KEYBOARD_COL1_PIN | KEYBOARD_COL2_PIN | 
                      KEYBOARD_COL3_PIN | KEYBOARD_COL4_PIN, 
                      GPIO_PIN_SET);
    
    // Based on test results, the actual mapping is:
    // Physical S16 -> detected as column 4 (PA4) -> Floor 1
    // Physical S15 -> detected as column 1 (PA8) -> Floor 2  
    // Physical S14 -> detected as column 3 (PA5) -> Floor 3 (already correct)
    // Physical S13 -> detected as column 2 (PA12) -> Floor 4
    
    // Scan Floor 1 button (Physical S16 at c4 = PA4)
    HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL4_PIN, GPIO_PIN_RESET);
    for(volatile int i = 0; i < 100; i++);
    if (HAL_GPIO_ReadPin(KEYBOARD_ROW_PORT, KEYBOARD_ROW_PIN) == GPIO_PIN_RESET) {
        key = KEY_S16;  // Floor 1
    }
    HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL4_PIN, GPIO_PIN_SET);
    
    if (key == KEY_NONE) {
        // Scan Floor 2 button (Physical S15 at c1 = PA8)
        HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL1_PIN, GPIO_PIN_RESET);
        for(volatile int i = 0; i < 100; i++);
        if (HAL_GPIO_ReadPin(KEYBOARD_ROW_PORT, KEYBOARD_ROW_PIN) == GPIO_PIN_RESET) {
            key = KEY_S15;  // Floor 2
        }
        HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL1_PIN, GPIO_PIN_SET);
    }
    
    if (key == KEY_NONE) {
        // Scan Floor 3 button (Physical S14 at c3 = PA5) - Already correct
        HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL3_PIN, GPIO_PIN_RESET);
        for(volatile int i = 0; i < 100; i++);
        if (HAL_GPIO_ReadPin(KEYBOARD_ROW_PORT, KEYBOARD_ROW_PIN) == GPIO_PIN_RESET) {
            key = KEY_S14;  // Floor 3
        }
        HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL3_PIN, GPIO_PIN_SET);
    }
    
    if (key == KEY_NONE) {
        // Scan Floor 4 button (Physical S13 at c2 = PA12)
        HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL2_PIN, GPIO_PIN_RESET);
        for(volatile int i = 0; i < 100; i++);
        if (HAL_GPIO_ReadPin(KEYBOARD_ROW_PORT, KEYBOARD_ROW_PIN) == GPIO_PIN_RESET) {
            key = KEY_S13;  // Floor 4 (not used currently)
        }
        HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL2_PIN, GPIO_PIN_SET);
    }
    
    return key;
}

// 检查按键是否按下
bool Keyboard_IsKeyPressed(uint8_t key) {
    return (g_keyboard.current_key == key && g_keyboard.key_stable);
}

// 将按键推入缓冲区
bool Keyboard_PushKey(uint8_t key) {
    if (g_keyboard.buffer_count >= KEYBOARD_BUFFER_SIZE) {
        return false;  // 缓冲区满
    }
    
    g_keyboard.key_buffer[g_keyboard.buffer_tail] = key;
    g_keyboard.buffer_tail = (g_keyboard.buffer_tail + 1) % KEYBOARD_BUFFER_SIZE;
    g_keyboard.buffer_count++;
    
    return true;
}

// 从缓冲区弹出按键
bool Keyboard_PopKey(uint8_t* key) {
    if (g_keyboard.buffer_count == 0) {
        return false;  // 缓冲区空
    }
    
    *key = g_keyboard.key_buffer[g_keyboard.buffer_head];
    g_keyboard.buffer_head = (g_keyboard.buffer_head + 1) % KEYBOARD_BUFFER_SIZE;
    g_keyboard.buffer_count--;
    
    return true;
}

// 检查缓冲区是否有按键
bool Keyboard_HasKey(void) {
    return g_keyboard.buffer_count > 0;
}

// 清空缓冲区
void Keyboard_ClearBuffer(void) {
    g_keyboard.buffer_head = 0;
    g_keyboard.buffer_tail = 0;
    g_keyboard.buffer_count = 0;
}

// 处理按键
void Keyboard_ProcessKey(uint8_t key) {
    if (key != KEY_NONE) {
        g_keyboard.total_key_presses++;
        g_keyboard.key_press_time = HAL_GetTick();
        
        // 推入缓冲区
        if (!Keyboard_PushKey(key)) {
            // 缓冲区满，清空后再推入
            Keyboard_ClearBuffer();
            Keyboard_PushKey(key);
        }
        
        // Update blackboard - COMMENTED OUT FOR TESTING
        // LocalBlackboard_SetKeyboardInput(key);
    }
}

// Handle floor request - STUB FOR TESTING
void Keyboard_HandleFloorRequest(uint8_t floor) {
    // STUB - Do nothing for testing
    (void)floor;
}

// Handle door control - STUB FOR TESTING
void Keyboard_HandleDoorControl(uint8_t cmd) {
    // STUB - Do nothing for testing
    (void)cmd;
}

// Handle emergency - STUB FOR TESTING
void Keyboard_HandleEmergency(void) {
    // STUB - Do nothing for testing
}

// GPIO Init - Modified for S13-S16 configuration
void Keyboard_GPIO_Init(void) {
    // GPIO is already initialized in gpio.c
    // Just set all columns HIGH initially
    HAL_GPIO_WritePin(KEYBOARD_COL_PORT, 
                      KEYBOARD_COL1_PIN | KEYBOARD_COL2_PIN | 
                      KEYBOARD_COL3_PIN | KEYBOARD_COL4_PIN, 
                      GPIO_PIN_SET);
}

// Set row level - NOT USED FOR S13-S16
void Keyboard_SetRow(uint8_t row) {
    // Not used in S13-S16 configuration
    (void)row;
}

// Read column status - NOT USED FOR S13-S16
uint8_t Keyboard_ReadCol(void) {
    // Not used in S13-S16 configuration
    return 0;
}

// Key mapping - NOT USED FOR S13-S16
uint8_t Keyboard_MapKey(uint8_t row, uint8_t col) {
    // Not used in S13-S16 configuration
    (void)row;
    (void)col;
    return KEY_NONE;
}

// 打印状态
void Keyboard_PrintStatus(void) {
    printf("=== Keyboard Status ===\n");
    printf("Current Key: %d\n", g_keyboard.current_key);
    printf("Key Stable: %s\n", g_keyboard.key_stable ? "Yes" : "No");
    printf("Buffer Count: %d\n", g_keyboard.buffer_count);
    printf("Total Presses: %ld\n", g_keyboard.total_key_presses);
    printf("Invalid Keys: %ld\n", g_keyboard.invalid_key_count);
    printf("======================\n");
}