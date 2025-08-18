#include "button_handler.h"
#include <string.h>

// 按钮数组（全局，供外部访问）
Button_t buttons[NUM_BUTTONS];

// 去抖动延时（毫秒）
#define DEBOUNCE_DELAY 50  // 增加到50ms去抖动，防止误触发
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
    
    // 2楼上行按钮
    buttons[1].port = BUTTON_UP_2_GPIO_Port;
    buttons[1].pin = BUTTON_UP_2_Pin;
    buttons[1].type = BUTTON_TYPE_UP;
    buttons[1].floor = 2;
    
    // 2楼下行按钮
    buttons[2].port = BUTTON_DOWN_2_GPIO_Port;
    buttons[2].pin = BUTTON_DOWN_2_Pin;
    buttons[2].type = BUTTON_TYPE_DOWN;
    buttons[2].floor = 2;
    
    // 3楼上行按钮
    buttons[3].port = BUTTON_UP_3_GPIO_Port;
    buttons[3].pin = BUTTON_UP_3_Pin;
    buttons[3].type = BUTTON_TYPE_UP;
    buttons[3].floor = 3;
    
    // 3楼下行按钮
    buttons[4].port = BUTTON_DOWN_3_GPIO_Port;
    buttons[4].pin = BUTTON_DOWN_3_Pin;
    buttons[4].type = BUTTON_TYPE_DOWN;
    buttons[4].floor = 3;
    
    // 初始化所有按钮状态
    for (int i = 0; i < NUM_BUTTONS; i++) {
        buttons[i].pressed = false;
        buttons[i].prev_state = false;  // 初始化为未按下状态（false表示引脚是高电平）
        buttons[i].debounce_time = 0;
        buttons[i].last_press_time = 0;
    }
}
// 中断标记（每个按钮一个标记）
static volatile bool button_interrupt_pending[NUM_BUTTONS] = {false};
static uint32_t button_press_count = 0;  // 按键次数计数

// 中断处理函数（从中断调用）
void Button_IRQHandler(uint8_t button_index) {
    if (button_index < NUM_BUTTONS) {
        button_interrupt_pending[button_index] = true;
        button_press_count++;
    }
}

// 处理按钮输入（主循环中处理去抖动和状态更新）
void Button_Process(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 处理中断标记
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (button_interrupt_pending[i]) {
            button_interrupt_pending[i] = false;
            
            Button_t* btn = &buttons[i];
            
            // 去抖动检查
            if ((current_time - btn->debounce_time) > DEBOUNCE_DELAY) {
                // 二次确认：连续读取3次，确保稳定
                int low_count = 0;
                for (int j = 0; j < 3; j++) {
                    if (HAL_GPIO_ReadPin(btn->port, btn->pin) == GPIO_PIN_RESET) {
                        low_count++;
                    }
                    HAL_Delay(1);  // 1ms间隔
                }
                
                // 只有3次都是低电平才认为按钮被按下
                if (low_count == 3) {
                    if (!btn->pressed) {
                        btn->pressed = true;
                        btn->last_press_time = current_time;
                        btn->debounce_time = current_time;
                        button_press_count++;  // 增加按键计数
                    }
                }
            }
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
        case GPIO_PIN_0:  Button_IRQHandler(2); break;  // PC0 - Floor 2 Down
        case GPIO_PIN_1:  Button_IRQHandler(1); break;  // PC1 - Floor 2 Up  
        case GPIO_PIN_2:  Button_IRQHandler(4); break;  // PC2 - Floor 3 Down
        case GPIO_PIN_3:  Button_IRQHandler(3); break;  // PC3 - Floor 3 Up
        case GPIO_PIN_5:  Button_IRQHandler(0); break;  // PC5 - Floor 1 Up
    }
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