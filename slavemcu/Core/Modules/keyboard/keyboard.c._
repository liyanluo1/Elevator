#include "keyboard.h"
#include "rs485_slave.h"
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
    
    // 处理缓冲区中的按键
    uint8_t buffered_key;
    if (Keyboard_PopKey(&buffered_key)) {
        // 在数字孪生模式下忽略物理按键
        if (!g_local_bb.digital_twin_mode) {
            switch (buffered_key) {
                case KEY_1:
                case KEY_2:
                case KEY_3:
                    Keyboard_HandleFloorRequest(buffered_key);
                    break;
                    
                case KEY_OPEN:
                    Keyboard_HandleDoorControl(KEY_OPEN);
                    break;
                    
                case KEY_CLOSE:
                    Keyboard_HandleDoorControl(KEY_CLOSE);
                    break;
                    
                case KEY_EMERGENCY:
                    Keyboard_HandleEmergency();
                    break;
                    
                default:
                    g_keyboard.invalid_key_count++;
                    break;
            }
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

// 读取键盘矩阵
uint8_t Keyboard_ReadMatrix(void) {
    uint8_t key = KEY_NONE;
    
    // 扫描4行
    for (uint8_t row = 0; row < 4; row++) {
        // 设置当前行为低电平
        Keyboard_SetRow(row);
        
        // 短暂延时让信号稳定
        for (volatile int i = 0; i < 100; i++);
        
        // 读取列
        uint8_t col_data = Keyboard_ReadCol();
        
        // 检查是否有按键按下
        if (col_data != 0x0F) {  // 有列被拉低
            // 找到具体的列
            for (uint8_t col = 0; col < 4; col++) {
                if (!(col_data & (1 << col))) {
                    key = Keyboard_MapKey(row, col);
                    break;
                }
            }
            break;
        }
    }
    
    // 恢复所有行为高电平
    HAL_GPIO_WritePin(KEYBOARD_ROW_PORT, 
                      KEYBOARD_ROW1_PIN | KEYBOARD_ROW2_PIN | 
                      KEYBOARD_ROW3_PIN | KEYBOARD_ROW4_PIN, 
                      GPIO_PIN_SET);
    
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
        
        // 更新黑板
        LocalBlackboard_SetKeyboardInput(key);
    }
}

// 处理楼层请求
void Keyboard_HandleFloorRequest(uint8_t floor) {
    if (floor >= 1 && floor <= MAX_FLOORS) {
        // 更新本地黑板
        LocalBlackboard_SetTargetFloor(floor);
        
        // 发送到主MCU
        RS485_Slave_SendKeyboardData(floor);
        
        // 推送事件
        LocalBlackboard_PushEvent(EVENT_TARGET_UPDATED, floor);
    }
}

// 处理门控制
void Keyboard_HandleDoorControl(uint8_t cmd) {
    switch (cmd) {
        case KEY_OPEN:
            // 推送开门事件
            LocalBlackboard_PushEvent(EVENT_OPEN_DOOR, 0);
            break;
            
        case KEY_CLOSE:
            // 推送关门事件
            LocalBlackboard_PushEvent(EVENT_CLOSE_DOOR, 0);
            break;
            
        default:
            break;
    }
}

// 处理紧急按钮
void Keyboard_HandleEmergency(void) {
    // 设置错误代码
    g_local_bb.error_code = 0x400;  // 紧急停止
    LocalBlackboard_MarkForSync(SYNC_FIELD_ERROR);
    
    // 发送紧急停止命令
    RS485_Slave_SendError(0x400);
    
    // 推送错误事件
    LocalBlackboard_PushEvent(EVENT_ERROR, 0x400);
}

// GPIO初始化
void Keyboard_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIO时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // 配置行GPIO为输出（默认高电平）
    GPIO_InitStruct.Pin = KEYBOARD_ROW1_PIN | KEYBOARD_ROW2_PIN | 
                         KEYBOARD_ROW3_PIN | KEYBOARD_ROW4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(KEYBOARD_ROW_PORT, &GPIO_InitStruct);
    
    // 设置所有行为高电平
    HAL_GPIO_WritePin(KEYBOARD_ROW_PORT, 
                      KEYBOARD_ROW1_PIN | KEYBOARD_ROW2_PIN | 
                      KEYBOARD_ROW3_PIN | KEYBOARD_ROW4_PIN, 
                      GPIO_PIN_SET);
    
    // 配置列GPIO为输入（带上拉）
    GPIO_InitStruct.Pin = KEYBOARD_COL1_PIN | KEYBOARD_COL2_PIN | 
                         KEYBOARD_COL3_PIN | KEYBOARD_COL4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEYBOARD_COL_PORT, &GPIO_InitStruct);
}

// 设置行电平
void Keyboard_SetRow(uint8_t row) {
    // 先将所有行设为高电平
    HAL_GPIO_WritePin(KEYBOARD_ROW_PORT, 
                      KEYBOARD_ROW1_PIN | KEYBOARD_ROW2_PIN | 
                      KEYBOARD_ROW3_PIN | KEYBOARD_ROW4_PIN, 
                      GPIO_PIN_SET);
    
    // 将选中的行设为低电平
    switch (row) {
        case 0:
            HAL_GPIO_WritePin(KEYBOARD_ROW_PORT, KEYBOARD_ROW1_PIN, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(KEYBOARD_ROW_PORT, KEYBOARD_ROW2_PIN, GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(KEYBOARD_ROW_PORT, KEYBOARD_ROW3_PIN, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(KEYBOARD_ROW_PORT, KEYBOARD_ROW4_PIN, GPIO_PIN_RESET);
            break;
    }
}

// 读取列状态
uint8_t Keyboard_ReadCol(void) {
    uint8_t col_data = 0;
    
    if (HAL_GPIO_ReadPin(KEYBOARD_COL_PORT, KEYBOARD_COL1_PIN) == GPIO_PIN_SET) {
        col_data |= (1 << 0);
    }
    if (HAL_GPIO_ReadPin(KEYBOARD_COL_PORT, KEYBOARD_COL2_PIN) == GPIO_PIN_SET) {
        col_data |= (1 << 1);
    }
    if (HAL_GPIO_ReadPin(KEYBOARD_COL_PORT, KEYBOARD_COL3_PIN) == GPIO_PIN_SET) {
        col_data |= (1 << 2);
    }
    if (HAL_GPIO_ReadPin(KEYBOARD_COL_PORT, KEYBOARD_COL4_PIN) == GPIO_PIN_SET) {
        col_data |= (1 << 3);
    }
    
    return col_data;
}

// 键值映射
uint8_t Keyboard_MapKey(uint8_t row, uint8_t col) {
    // 4x4键盘映射表
    // 可根据实际键盘布局调整
    const uint8_t key_map[4][4] = {
        {KEY_1,         KEY_2,          KEY_3,          KEY_OPEN},      // 第1行
        {KEY_NONE,      KEY_NONE,       KEY_NONE,       KEY_CLOSE},     // 第2行
        {KEY_NONE,      KEY_NONE,       KEY_NONE,       KEY_NONE},      // 第3行
        {KEY_EMERGENCY, KEY_NONE,       KEY_NONE,       KEY_NONE}       // 第4行
    };
    
    if (row < 4 && col < 4) {
        return key_map[row][col];
    }
    
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