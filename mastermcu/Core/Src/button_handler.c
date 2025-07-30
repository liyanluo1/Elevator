#include "button_handler.h"
#include "blackboard.h"
#include <string.h>

// 前向声明
static void HandleButtonPress(Button_t* btn);

// 按钮数组
static Button_t buttons[NUM_BUTTONS];

// 去抖动延时（毫秒）
#define DEBOUNCE_DELAY 50

// 初始化按钮
void Button_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIO时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // 配置按钮数组
    // 1楼上行按钮
    buttons[0].port = BUTTON_UP_1_GPIO_Port;
    buttons[0].pin = BUTTON_UP_1_Pin;
    buttons[0].type = BUTTON_TYPE_UP;
    buttons[0].floor = FLOOR_1;
    
    // 1楼下行按钮（实际不使用，但保留接口）
    buttons[1].port = BUTTON_DOWN_1_GPIO_Port;
    buttons[1].pin = BUTTON_DOWN_1_Pin;
    buttons[1].type = BUTTON_TYPE_DOWN;
    buttons[1].floor = FLOOR_1;
    
    // 2楼上行按钮
    buttons[2].port = BUTTON_UP_2_GPIO_Port;
    buttons[2].pin = BUTTON_UP_2_Pin;
    buttons[2].type = BUTTON_TYPE_UP;
    buttons[2].floor = FLOOR_2;
    
    // 2楼下行按钮
    buttons[3].port = BUTTON_DOWN_2_GPIO_Port;
    buttons[3].pin = BUTTON_DOWN_2_Pin;
    buttons[3].type = BUTTON_TYPE_DOWN;
    buttons[3].floor = FLOOR_2;
    
    // 3楼上行按钮（实际不使用，但保留接口）
    buttons[4].port = BUTTON_UP_3_GPIO_Port;
    buttons[4].pin = BUTTON_UP_3_Pin;
    buttons[4].type = BUTTON_TYPE_UP;
    buttons[4].floor = FLOOR_3;
    
    // 3楼下行按钮
    buttons[5].port = BUTTON_DOWN_3_GPIO_Port;
    buttons[5].pin = BUTTON_DOWN_3_Pin;
    buttons[5].type = BUTTON_TYPE_DOWN;
    buttons[5].floor = FLOOR_3;
    
    // 配置所有按钮GPIO为输入模式，带上拉电阻
    for (int i = 0; i < NUM_BUTTONS; i++) {
        GPIO_InitStruct.Pin = buttons[i].pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(buttons[i].port, &GPIO_InitStruct);
        
        // 初始化状态
        buttons[i].pressed = false;
        buttons[i].prev_state = true;  // 上拉，默认高电平
        buttons[i].debounce_time = 0;
        buttons[i].last_press_time = 0;
    }
    
    // 配置LED输出引脚（假设每个按钮有对应的LED）
    // 这里需要根据实际硬件配置
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    // 配置LED引脚...（根据实际硬件添加）
}

// 处理按钮输入
void Button_Process(void) {
    uint32_t current_time = HAL_GetTick();
    
    for (int i = 0; i < NUM_BUTTONS; i++) {
        Button_t* btn = &buttons[i];
        
        // 读取当前按钮状态（低电平表示按下）
        bool current_state = HAL_GPIO_ReadPin(btn->port, btn->pin) == GPIO_PIN_RESET;
        
        // 状态变化检测
        if (current_state != btn->prev_state) {
            btn->debounce_time = current_time;
        }
        
        // 去抖动处理
        if ((current_time - btn->debounce_time) > DEBOUNCE_DELAY) {
            // 状态已稳定
            if (current_state && !btn->pressed) {
                // 按钮刚被按下
                btn->pressed = true;
                btn->last_press_time = current_time;
                
                // 处理按钮按下事件
                HandleButtonPress(btn);
            } else if (!current_state && btn->pressed) {
                // 按钮刚被释放
                btn->pressed = false;
            }
        }
        
        btn->prev_state = current_state;
    }
    
    // 更新LED状态
    Button_UpdateLEDs();
}

// 处理按钮按下事件（内部函数）
static void HandleButtonPress(Button_t* btn) {
    // 根据楼层限制，忽略无效按钮
    if (btn->floor == FLOOR_1 && btn->type == BUTTON_TYPE_DOWN) {
        return;  // 1楼没有下行
    }
    if (btn->floor == FLOOR_3 && btn->type == BUTTON_TYPE_UP) {
        return;  // 3楼没有上行
    }
    
    // 数字孪生模式检查
    if (g_blackboard.digital_twin_mode) {
        return;  // 忽略物理按钮
    }
    
    // 设置呼叫状态（转换为1-3楼层编号）
    uint8_t floor_num = btn->floor + 1;
    Blackboard_SetPendingCall(floor_num, true);
    
    // 如果电梯在当前楼层且门已关闭，立即开门
    if (g_blackboard.current_floor == btn->floor && 
        g_blackboard.state == ELEVATOR_IDLE &&
        g_blackboard.door.is_closed) {
        Blackboard_PushEvent(EVENT_OPEN_DOOR, btn->floor);
    }
}

// 检查按钮是否被按下
bool Button_IsPressed(uint8_t button_index) {
    if (button_index < NUM_BUTTONS) {
        return buttons[button_index].pressed;
    }
    return false;
}

// 设置按钮LED状态
void Button_SetLED(uint8_t floor, ButtonType_t type, bool state) {
    // 根据实际硬件配置实现LED控制
    // 这里需要根据实际的LED引脚配置来实现
    
    // 示例：假设LED连接到GPIOB
    // GPIO_TypeDef* led_port = GPIOB;
    // uint16_t led_pin = GPIO_PIN_0;  // 根据floor和type计算实际引脚
    
    // if (state) {
    //     HAL_GPIO_WritePin(led_port, led_pin, GPIO_PIN_SET);
    // } else {
    //     HAL_GPIO_WritePin(led_port, led_pin, GPIO_PIN_RESET);
    // }
}

// 更新所有LED状态
void Button_UpdateLEDs(void) {
    // 更新上行呼叫LED
    for (int floor = 0; floor < MAX_FLOORS - 1; floor++) {
        bool has_call = g_blackboard.floors[floor].call_up;
        Button_SetLED(floor, BUTTON_TYPE_UP, has_call);
    }
    
    // 更新下行呼叫LED
    for (int floor = 1; floor < MAX_FLOORS; floor++) {
        bool has_call = g_blackboard.floors[floor].call_down;
        Button_SetLED(floor, BUTTON_TYPE_DOWN, has_call);
    }
}