// led.c
#include "led.h"

// Define LED pin
#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_1  // LED -> PA1 (output)

void LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIOA clock (if not already)
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Output for LED, push-pull, initial low (off)
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);  // Off
}

void LED_On(void) {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

void LED_Off(void) {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

void LED_Toggle(void) {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}

void LED_Flash(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);  // On
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);  // Off
        HAL_Delay(200);
    }
    HAL_Delay(500);  // Pause after flashes
}
