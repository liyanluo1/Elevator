// keyboard.h
#ifndef KEYBOARD_H
#define KEYBOARD_H

#include "stm32f1xx_hal.h"  // Adjust for your STM32 series, e.g., stm32f1xx_hal.h

void Keyboard_Init(void);
uint8_t Keyboard_Scan(void);  // Returns 0 if no key, 1 for S1, 2 for S2, 3 for S5, 4 for S6

#endif
