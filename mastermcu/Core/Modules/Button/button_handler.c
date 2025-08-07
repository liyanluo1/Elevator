#include "button_handler.h"
#include <string.h>

// 按钮数组
static Button_t buttons[NUM_BUTTONS];

// 去抖动延时（毫秒）
#define DEBOUNCE_DELAY 20  // 20ms去抖动，更快响应
#define BUTTON_HOLD_TIME 300  // 按钮保持时间（显示用）

// 初始化按钮
void Button_Init(void) {
    // 注意：GPIO初始化应该在CubeMX生成的MX_GPIO_Init()中完成
    // 这里只初始化按钮状态结构体
    
    // 配置按钮数组
    // 1楼上行按钮
    buttons[0].port = BUTTON_UP_1_GPIO_Port;
    buttons[0].pin = BUTTON_UP_1_Pin;
    buttons[0].type = BUTTON_TYPE_UP;
    buttons[0].floor = 1;
    
    // 1楼下行按钮
    buttons[1].port = BUTTON_DOWN_1_GPIO_Port;
    buttons[1].pin = BUTTON_DOWN_1_Pin;
    buttons[1].type = BUTTON_TYPE_DOWN;
    buttons[1].floor = 1;
    
    // 2楼上行按钮
    buttons[2].port = BUTTON_UP_2_GPIO_Port;
    buttons[2].pin = BUTTON_UP_2_Pin;
    buttons[2].type = BUTTON_TYPE_UP;
    buttons[2].floor = 2;
    
    // 2楼下行按钮
    buttons[3].port = BUTTON_DOWN_2_GPIO_Port;
    buttons[3].pin = BUTTON_DOWN_2_Pin;
    buttons[3].type = BUTTON_TYPE_DOWN;
    buttons[3].floor = 2;
    
    // 3楼上行按钮
    buttons[4].port = BUTTON_UP_3_GPIO_Port;
    buttons[4].pin = BUTTON_UP_3_Pin;
    buttons[4].type = BUTTON_TYPE_UP;
    buttons[4].floor = 3;
    
    // 3楼下行按钮
    buttons[5].port = BUTTON_DOWN_3_GPIO_Port;
    buttons[5].pin = BUTTON_DOWN_3_Pin;
    buttons[5].type = BUTTON_TYPE_DOWN;
    buttons[5].floor = 3;
    
    // 初始化所有按钮状态
    for (int i = 0; i < NUM_BUTTONS; i++) {
        buttons[i].pressed = false;
        buttons[i].prev_state = false;  // 初始化为未按下状态（false表示引脚是高电平）
        buttons[i].debounce_time = 0;
        buttons[i].last_press_time = 0;
    }
}
// 中断标记（每个按钮一个标记）
static volatile uint8_t button_interrupt_flags = 0;
static volatile uint32_t interrupt_count = 0;
static uint32_t button_press_count = 0;  // 按键次数计数

// 处理按钮输入（主循环中处理去抖动和状态更新）
void Button_Process(void) {
    uint32_t current_time = HAL_GetTick();
    static uint32_t last_scan_time = 0;
    
    // 改为轮询方式，每5ms扫描一次所有按钮（更快响应）
    if (current_time - last_scan_time >= 5) {
        last_scan_time = current_time;
        
        // 直接扫描所有按钮（不依赖中断）
        for (int i = 0; i < NUM_BUTTONS; i++) {
            Button_t* btn = &buttons[i];
            
            // 读取当前状态
            bool is_pressed = HAL_GPIO_ReadPin(btn->port, btn->pin) == GPIO_PIN_RESET;
            
            // 状态变化检测
            if (is_pressed != btn->prev_state) {
                btn->debounce_time = current_time;
            }
            
            // 去抖动处理
            if ((current_time - btn->debounce_time) > DEBOUNCE_DELAY) {
                if (is_pressed && !btn->pressed) {
                    btn->pressed = true;
                    btn->last_press_time = current_time;
                    button_press_count++;  // 增加按键计数
                } else if (!is_pressed && btn->pressed) {
                    // 按钮释放不处理（电梯外呼只关心按下）
                }
            }
            
            btn->prev_state = is_pressed;
        }
    }
    
    // 自动复位超时的按钮
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (buttons[i].pressed) {
            if (current_time - buttons[i].last_press_time > BUTTON_HOLD_TIME) {
                buttons[i].pressed = false;
            }
        }
    }
}

// 检查按钮是否被按下
bool Button_IsPressed(uint8_t button_index) {
    if (button_index >= NUM_BUTTONS) {
        return false;
    }
    return buttons[button_index].pressed;
}

// 获取指定楼层的上行按钮状态
bool Button_GetUpState(uint8_t floor) {
    if (floor < 1 || floor > 3) return false;
    
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (buttons[i].floor == floor && buttons[i].type == BUTTON_TYPE_UP) {
            return buttons[i].pressed;
        }
    }
    return false;
}

// 获取指定楼层的下行按钮状态
bool Button_GetDownState(uint8_t floor) {
    if (floor < 1 || floor > 3) return false;
    
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (buttons[i].floor == floor && buttons[i].type == BUTTON_TYPE_DOWN) {
            return buttons[i].pressed;
        }
    }
    return false;
}

// 获取当前按下的按钮总数
uint8_t Button_GetPressedCount(void) {
    uint8_t count = 0;
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (buttons[i].pressed) {
            count++;
        }
    }
    return count;
}

// 获取所有按钮状态
void Button_GetAllStates(bool* up_states, bool* down_states) {
    if (up_states != NULL) {
        for (int floor = 1; floor <= 3; floor++) {
            up_states[floor - 1] = Button_GetUpState(floor);
        }
    }
    
    if (down_states != NULL) {
        for (int floor = 1; floor <= 3; floor++) {
            down_states[floor - 1] = Button_GetDownState(floor);
        }
    }
}



// 外部中断回调（优化版本 - 极简快速）
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // 快速标记哪个引脚触发了中断（<1us）
    switch(GPIO_Pin) {
        case GPIO_PIN_0:  button_interrupt_flags |= (1 << 0); break;  // PC0
        case GPIO_PIN_1:  button_interrupt_flags |= (1 << 1); break;  // PC1
        case GPIO_PIN_2:  button_interrupt_flags |= (1 << 2); break;  // PC2
        case GPIO_PIN_3:  button_interrupt_flags |= (1 << 3); break;  // PC3
        case GPIO_PIN_5:  button_interrupt_flags |= (1 << 4); break;  // PC5
        case GPIO_PIN_15: button_interrupt_flags |= (1 << 5); break;  // PC15
    }
    interrupt_count++;  // 调试计数
    // 中断结束 - 总时间 < 1us
}

// 获取中断计数（调试用）
uint32_t Button_GetInterruptCount(void) {
    return button_press_count;  // 返回实际按键次数而不是中断次数
}

// LED控制函数（如果需要）
void Button_SetLED(uint8_t floor, ButtonType_t type, bool state) {
    // 根据实际硬件连接实现LED控制
    // 这里留空，等待具体硬件配置
}

void Button_UpdateLEDs(void) {
    // 更新所有LED状态
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (buttons[i].pressed) {
            Button_SetLED(buttons[i].floor, buttons[i].type, true);
        } else {
            Button_SetLED(buttons[i].floor, buttons[i].type, false);
        }
    }
}