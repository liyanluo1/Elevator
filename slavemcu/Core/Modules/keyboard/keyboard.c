#include "keyboard.h"
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
    g_keyboard.interrupt_flag = false;
    g_keyboard.interrupt_time = 0;
    
    // 初始化GPIO
    Keyboard_GPIO_Init();
}

// 主处理函数 - 中断模式
void Keyboard_Handler(void) {
    // 检查中断标志
    if (g_keyboard.interrupt_flag) {
        g_keyboard.interrupt_flag = false;
        
        uint32_t current_time = HAL_GetTick();
        
        // 去抖处理 - 两次按键间隔至少 KEYBOARD_DEBOUNCE_TIME
        if (current_time - g_keyboard.last_scan_time >= KEYBOARD_DEBOUNCE_TIME) {
            g_keyboard.last_scan_time = current_time;
            
            // 删除延时，GPIO读取足够快
            
            // 扫描键盘确定哪个按键被按下
            uint8_t key = Keyboard_ScanInterrupt();
            
            // 处理按键 - 允许重复按键
            if (key != KEY_NONE) {
                // 只在按键不同或超过释放时间后才处理
                if (key != g_keyboard.last_key || 
                    (current_time - g_keyboard.key_press_time) > KEYBOARD_RELEASE_TIME) {
                    Keyboard_ProcessKey(key);
                    g_keyboard.last_key = key;
                    // 只在按键被处理时更新时间
                    g_keyboard.key_press_time = current_time;
                }
            }
        }
    }
    
    // 检查按键释放
    uint32_t current_time = HAL_GetTick();
    if (g_keyboard.last_key != KEY_NONE) {
        if (current_time - g_keyboard.key_press_time > KEYBOARD_RELEASE_TIME) {
            g_keyboard.last_key = KEY_NONE;
        }
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

// Scan keyboard when interrupt triggered
uint8_t Keyboard_ScanInterrupt(void) {
    uint8_t key = KEY_NONE;
    uint8_t confirm_count = 0;
    
    // First set all columns HIGH to prepare for scanning
    HAL_GPIO_WritePin(KEYBOARD_COL_PORT, 
                      KEYBOARD_COL1_PIN | KEYBOARD_COL2_PIN | 
                      KEYBOARD_COL3_PIN | KEYBOARD_COL4_PIN, 
                      GPIO_PIN_SET);
    
    // 无需等待，GPIO切换很快
    
    // Physical wiring mapping:
    // S16 -> PA4 -> Floor 1
    // S15 -> PA8 -> Floor 2
    // S14 -> PA5 -> Floor 3
    // S13 -> PA12 -> Not used
    
    // Test each column one by one with confirmation
    
    // Test PA4 (COL4_PIN) for S16 -> Floor 1
    HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL4_PIN, GPIO_PIN_RESET);  // PA4
    // 无需延时
    // 连续读取3次确认
    confirm_count = 0;
    for (int i = 0; i < 3; i++) {
        if (HAL_GPIO_ReadPin(KEYBOARD_ROW_PORT, KEYBOARD_ROW_PIN) == GPIO_PIN_RESET) {
            confirm_count++;
        }
        // 无需延时
    }
    if (confirm_count >= 1) {  // 放宽条件：1次即可触发
        key = KEY_S16;  // Floor 1
    }
    HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL4_PIN, GPIO_PIN_SET);
    
    if (key == KEY_NONE) {
        // Test PA8 (COL1_PIN) for S15 -> Floor 2
        HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL1_PIN, GPIO_PIN_RESET);  // PA8
        for(volatile int delay = 0; delay < 1000; delay++);
        confirm_count = 0;
        for (int i = 0; i < 3; i++) {
            if (HAL_GPIO_ReadPin(KEYBOARD_ROW_PORT, KEYBOARD_ROW_PIN) == GPIO_PIN_RESET) {
                confirm_count++;
            }
            // 无需延时
        }
        if (confirm_count >= 1) {  // 放宽条件：1次即可触发
            key = KEY_S15;  // Floor 2
        }
        HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL1_PIN, GPIO_PIN_SET);
    }
    
    if (key == KEY_NONE) {
        // Test PA5 (COL3_PIN) for S14 -> Floor 3
        HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL3_PIN, GPIO_PIN_RESET);  // PA5
        for(volatile int delay = 0; delay < 1000; delay++);
        confirm_count = 0;
        for (int i = 0; i < 3; i++) {
            if (HAL_GPIO_ReadPin(KEYBOARD_ROW_PORT, KEYBOARD_ROW_PIN) == GPIO_PIN_RESET) {
                confirm_count++;
            }
            // 无需延时
        }
        if (confirm_count >= 1) {  // 放宽条件：1次即可触发
            key = KEY_S14;  // Floor 3
        }
        HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL3_PIN, GPIO_PIN_SET);
    }
    
    if (key == KEY_NONE) {
        // Test PA12 (COL2_PIN) for S13 -> Not used
        HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL2_PIN, GPIO_PIN_RESET);  // PA12
        for(volatile int delay = 0; delay < 1000; delay++);
        confirm_count = 0;
        for (int i = 0; i < 3; i++) {
            if (HAL_GPIO_ReadPin(KEYBOARD_ROW_PORT, KEYBOARD_ROW_PIN) == GPIO_PIN_RESET) {
                confirm_count++;
            }
            // 无需延时
        }
        if (confirm_count >= 1) {  // 放宽条件：1次即可触发
            key = KEY_S13;  // S13 detected but not used in elevator logic
        }
        HAL_GPIO_WritePin(KEYBOARD_COL_PORT, KEYBOARD_COL2_PIN, GPIO_PIN_SET);
    }
    
    // 等待按键释放
    if (key != KEY_NONE) {
        // 等待按键释放（最多等10ms）- 进一步缩短
        uint32_t wait_start = HAL_GetTick();
        while ((HAL_GetTick() - wait_start) < 10) {
            // 设置所有列为HIGH
            HAL_GPIO_WritePin(KEYBOARD_COL_PORT, 
                              KEYBOARD_COL1_PIN | KEYBOARD_COL2_PIN | 
                              KEYBOARD_COL3_PIN | KEYBOARD_COL4_PIN, 
                              GPIO_PIN_SET);
            // 无需延时
            
            // 检查是否释放
            if (HAL_GPIO_ReadPin(KEYBOARD_ROW_PORT, KEYBOARD_ROW_PIN) == GPIO_PIN_SET) {
                // 按键已释放
                // 无需额外延时
                break;
            }
        }
    }
    
    // After scanning, set all columns LOW again for next interrupt
    HAL_GPIO_WritePin(KEYBOARD_COL_PORT, 
                      KEYBOARD_COL1_PIN | KEYBOARD_COL2_PIN | 
                      KEYBOARD_COL3_PIN | KEYBOARD_COL4_PIN, 
                      GPIO_PIN_RESET);
    
    return key;
}

// Read keyboard matrix - Not used in interrupt mode
uint8_t Keyboard_ReadMatrix(void) {
    // Not used in interrupt mode
    return KEY_NONE;
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
        
    }
}


// GPIO Init
void Keyboard_GPIO_Init(void) {
    // GPIO is already initialized in gpio.c
    // Set all columns LOW initially for interrupt mode
    // When any button is pressed, it will pull PA11 LOW and trigger interrupt
    HAL_GPIO_WritePin(KEYBOARD_COL_PORT, 
                      KEYBOARD_COL1_PIN | KEYBOARD_COL2_PIN | 
                      KEYBOARD_COL3_PIN | KEYBOARD_COL4_PIN, 
                      GPIO_PIN_RESET);
}


// 中断处理函数
void Keyboard_IRQHandler(void) {
    // 设置中断标志
    g_keyboard.interrupt_flag = true;
    g_keyboard.interrupt_time = HAL_GetTick();
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