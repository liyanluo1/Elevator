#ifndef GPIO_UTILS_H
#define GPIO_UTILS_H

#include "stm32f1xx_hal.h"  // STM32F1系列
#include <stdbool.h>

/* GPIO配置结构体 */
typedef struct {
    GPIO_TypeDef* port;     // GPIO端口
    uint16_t pin;          // GPIO引脚
    uint32_t mode;         // GPIO模式
    uint32_t pull;         // 上下拉配置
    uint32_t speed;        // GPIO速度
} GPIO_Config_t;

/* 简化的GPIO初始化 */
void GPIO_Init_Pin(GPIO_TypeDef* port, uint16_t pin, uint32_t mode, uint32_t pull);
void GPIO_Init_Pins(const GPIO_Config_t* configs, uint8_t count);

/* GPIO读写操作 */
static inline bool GPIO_Read(GPIO_TypeDef* port, uint16_t pin) {
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET);
}

static inline void GPIO_Write(GPIO_TypeDef* port, uint16_t pin, bool state) {
    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void GPIO_Toggle(GPIO_TypeDef* port, uint16_t pin) {
    HAL_GPIO_TogglePin(port, pin);
}

/* 批量GPIO操作 */
uint32_t GPIO_ReadPort(GPIO_TypeDef* port);
void GPIO_WritePort(GPIO_TypeDef* port, uint32_t value);

/* 中断配置助手 */
void GPIO_ConfigInterrupt(GPIO_TypeDef* port, uint16_t pin, 
                         uint32_t trigger, uint32_t priority);

/* LED控制宏 - 适用于低电平点亮的LED */
#define LED_ON(port, pin)   GPIO_Write(port, pin, false)
#define LED_OFF(port, pin)  GPIO_Write(port, pin, true)
#define LED_TOGGLE(port, pin) GPIO_Toggle(port, pin)

/* 按钮读取宏 - 适用于低电平按下的按钮 */
#define BUTTON_PRESSED(port, pin) (!GPIO_Read(port, pin))

#endif /* GPIO_UTILS_H */