#ifndef __BUTTON_HANDLER_H
#define __BUTTON_HANDLER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// 按钮GPIO配置 - 根据实际硬件连接更新
// 1楼
#define BUTTON_UP_1_GPIO_Port   GPIOC
#define BUTTON_UP_1_Pin         GPIO_PIN_5    // PC5 (原来是1楼下行，现在改为1楼上行)
#define BUTTON_DOWN_1_GPIO_Port GPIOC
#define BUTTON_DOWN_1_Pin       GPIO_PIN_15   // PC15 (不使用，但保留定义避免编译错误)

// 2楼
#define BUTTON_UP_2_GPIO_Port   GPIOC
#define BUTTON_UP_2_Pin         GPIO_PIN_1    // PC1
#define BUTTON_DOWN_2_GPIO_Port GPIOC
#define BUTTON_DOWN_2_Pin       GPIO_PIN_0    // PC0

// 3楼
#define BUTTON_UP_3_GPIO_Port   GPIOC
#define BUTTON_UP_3_Pin         GPIO_PIN_3    // PC3
#define BUTTON_DOWN_3_GPIO_Port GPIOC
#define BUTTON_DOWN_3_Pin       GPIO_PIN_2    // PC2

// 按钮类型
typedef enum {
    BUTTON_TYPE_UP = 0,
    BUTTON_TYPE_DOWN = 1
} ButtonType_t;

// 按钮状态
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    ButtonType_t type;
    uint8_t floor;
    bool pressed;
    bool prev_state;
    uint32_t debounce_time;
    uint32_t last_press_time;
} Button_t;

// 按钮数组大小
#define NUM_BUTTONS 6

// 函数声明
void Button_Init(void);
void Button_Process(void);
bool Button_IsPressed(uint8_t button_index);
void Button_SetLED(uint8_t floor, ButtonType_t type, bool state);
void Button_UpdateLEDs(void);

// 新增：获取按钮状态的函数
bool Button_GetUpState(uint8_t floor);
bool Button_GetDownState(uint8_t floor);
uint8_t Button_GetPressedCount(void);
void Button_GetAllStates(bool* up_states, bool* down_states);

// 外部中断回调
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// 调试函数
uint32_t Button_GetInterruptCount(void);

#endif /* __BUTTON_HANDLER_H */