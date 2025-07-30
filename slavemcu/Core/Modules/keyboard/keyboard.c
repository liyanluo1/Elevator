// keyboard.c
#include "keyboard.h"

// Define pins
#define ROW1_PORT GPIOA
#define ROW1_PIN GPIO_PIN_11  // r1 -> PA11 (input)
#define ROW2_PORT GPIOA
#define ROW2_PIN GPIO_PIN_4   // r2 -> PA4 (input)

#define COL4_PORT GPIOA
#define COL4_PIN GPIO_PIN_5   // c4 -> PA5 (output)
#define COL3_PORT GPIOA
#define COL3_PIN GPIO_PIN_12  // c3 -> PA12 (output)

void Keyboard_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIOA clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Input rows with pull-up
    GPIO_InitStruct.Pin = ROW1_PIN | ROW2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Output columns, push-pull, initial high
    GPIO_InitStruct.Pin = COL4_PIN | COL3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Set columns high initially
    HAL_GPIO_WritePin(COL4_PORT, COL4_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL3_PORT, COL3_PIN, GPIO_PIN_SET);
}

uint8_t Keyboard_Scan(void) {
    uint8_t key = 0;

    // Array for columns: index 0 = c4, 1 = c3
    GPIO_TypeDef* colPorts[] = {COL4_PORT, COL3_PORT};
    uint16_t colPins[] = {COL4_PIN, COL3_PIN};

    // Array for rows: index 0 = r1, 1 = r2
    GPIO_TypeDef* rowPorts[] = {ROW1_PORT, ROW2_PORT};
    uint16_t rowPins[] = {ROW1_PIN, ROW2_PIN};

    for (int col = 0; col < 2; col++) {
        // Set current column low
        HAL_GPIO_WritePin(colPorts[col], colPins[col], GPIO_PIN_RESET);

        for (int row = 0; row < 2; row++) {
            if (HAL_GPIO_ReadPin(rowPorts[row], rowPins[row]) == GPIO_PIN_RESET) {
                // Key pressed: map to 1-4
                // col 0 (c4): row0=1 (S1), row1=3 (S5)
                // col 1 (c3): row0=2 (S2), row1=4 (S6)
                key = (row * 2) + (col == 0 ? 1 : 2);
                // Simple debounce
                HAL_Delay(20);
                while (HAL_GPIO_ReadPin(rowPorts[row], rowPins[row]) == GPIO_PIN_RESET);  // Wait for release
                break;
            }
        }

        // Set column back high
        HAL_GPIO_WritePin(colPorts[col], colPins[col], GPIO_PIN_SET);

        if (key) break;  // Exit if key found
    }

    return key;
}
