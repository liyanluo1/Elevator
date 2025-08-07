#include "gpio_utils.h"

/* 简化的单个GPIO初始化 */
void GPIO_Init_Pin(GPIO_TypeDef* port, uint16_t pin, uint32_t mode, uint32_t pull) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能对应的GPIO时钟
    if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    #ifdef GPIOF
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    #endif
    #ifdef GPIOG
    else if (port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
    #endif
    
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

/* 批量GPIO初始化 */
void GPIO_Init_Pins(const GPIO_Config_t* configs, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        
        // 使能时钟
        if (configs[i].port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
        else if (configs[i].port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
        else if (configs[i].port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
        else if (configs[i].port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
        else if (configs[i].port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
        #ifdef GPIOF
        else if (configs[i].port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
        #endif
        #ifdef GPIOG
        else if (configs[i].port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
        #endif
        
        GPIO_InitStruct.Pin = configs[i].pin;
        GPIO_InitStruct.Mode = configs[i].mode;
        GPIO_InitStruct.Pull = configs[i].pull;
        GPIO_InitStruct.Speed = configs[i].speed;
        
        HAL_GPIO_Init(configs[i].port, &GPIO_InitStruct);
    }
}

/* 读取整个端口 */
uint32_t GPIO_ReadPort(GPIO_TypeDef* port) {
    return port->IDR;
}

/* 写入整个端口 */
void GPIO_WritePort(GPIO_TypeDef* port, uint32_t value) {
    port->ODR = value;
}

/* 配置GPIO中断 */
void GPIO_ConfigInterrupt(GPIO_TypeDef* port, uint16_t pin, 
                         uint32_t trigger, uint32_t priority) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = trigger;  // GPIO_MODE_IT_RISING等
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
    
    // 确定中断号
    IRQn_Type irqn;
    uint8_t pin_num = 0;
    
    // 计算引脚号
    uint16_t temp_pin = pin;
    while (temp_pin >>= 1) {
        pin_num++;
    }
    
    // 根据引脚号选择中断
    if (pin_num <= 4) {
        irqn = EXTI0_IRQn + pin_num;
    } else if (pin_num <= 9) {
        irqn = EXTI9_5_IRQn;
    } else {
        irqn = EXTI15_10_IRQn;
    }
    
    HAL_NVIC_SetPriority(irqn, priority, 0);
    HAL_NVIC_EnableIRQ(irqn);
}