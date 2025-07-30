#ifndef __BUTTON_HANDLER_H
#define __BUTTON_HANDLER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// 按钮GPIO配置
#define BUTTON_UP_1_GPIO_Port   GPIOA
#define BUTTON_UP_1_Pin         GPIO_PIN_0
#define BUTTON_DOWN_1_GPIO_Port GPIOA
#define BUTTON_DOWN_1_Pin       GPIO_PIN_1

#define BUTTON_UP_2_GPIO_Port   GPIOA
#define BUTTON_UP_2_Pin         GPIO_PIN_2
#define BUTTON_DOWN_2_GPIO_Port GPIOA
#define BUTTON_DOWN_2_Pin       GPIO_PIN_3

#define BUTTON_UP_3_GPIO_Port   GPIOA
#define BUTTON_UP_3_Pin         GPIO_PIN_4
#define BUTTON_DOWN_3_GPIO_Port GPIOA
#define BUTTON_DOWN_3_Pin       GPIO_PIN_5

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

#endif /* __BUTTON_HANDLER_H */