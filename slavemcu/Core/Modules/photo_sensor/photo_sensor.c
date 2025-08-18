#include "photo_sensor.h"

/* Private variables */
static volatile uint32_t trigger_count = 0;
static volatile uint32_t last_trigger_time = 0;

/**
 * @brief Initialize photo sensor on PB5 with interrupt
 */
void PhotoSensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIOB clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure PB5 as input with pull-up */
    GPIO_InitStruct.Pin = PHOTO_SENSOR_GPIO_PIN;  // GPIO_PIN_5
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;   /* Interrupt on rising edge only */
    GPIO_InitStruct.Pull = GPIO_PULLUP;           /* Internal pull-up resistor */
    HAL_GPIO_Init(PHOTO_SENSOR_GPIO_PORT, &GPIO_InitStruct);
    
    /* Enable and set EXTI9_5 interrupt priority (PB5 uses this) */
    HAL_NVIC_SetPriority(PHOTO_SENSOR_IRQn, 2, 0);  // EXTI9_5_IRQn
    HAL_NVIC_EnableIRQ(PHOTO_SENSOR_IRQn);
    
    /* Reset counter */
    trigger_count = 0;
    last_trigger_time = 0;
}

/**
 * @brief Get current state of photo sensor
 * @return PHOTO_SENSOR_BLOCKED if object detected, PHOTO_SENSOR_CLEAR if no object
 */
photo_sensor_state_t PhotoSensor_GetState(void)
{
    /* Read GPIO pin state */
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(PHOTO_SENSOR_GPIO_PORT, PHOTO_SENSOR_GPIO_PIN);
    
    /* White line (Dark-ON): LOW = object detected, HIGH = no object */
    if (pin_state == GPIO_PIN_RESET) {
        return PHOTO_SENSOR_BLOCKED;
    } else {
        return PHOTO_SENSOR_CLEAR;
    }
}

/**
 * @brief Get trigger count
 * @return Number of times sensor was triggered
 */
uint32_t PhotoSensor_GetTriggerCount(void)
{
    return trigger_count;
}

/**
 * @brief Reset trigger count
 */
void PhotoSensor_ResetCount(void)
{
    trigger_count = 0;
}

/**
 * @brief Photo sensor interrupt handler
 * @note Called from HAL_GPIO_EXTI_Callback
 */
void PhotoSensor_IRQHandler(void)
{
    /* Simple debounce: ignore triggers within 50ms */
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_trigger_time > 50) {
        trigger_count++;
        last_trigger_time = current_time;
        
        /* Call user callback */
        PhotoSensor_TriggerCallback();
    }
}


/**
 * @brief Weak callback function - override in main.c
 */
__weak void PhotoSensor_TriggerCallback(void)
{
    /* User can override this function */
}