// led.h
#ifndef LED_H
#define LED_H

#include "stm32f1xx_hal.h"  // Adjust for your STM32 series

void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
void LED_Flash(uint8_t times);  // Flash LED 'times' times, each flash 200ms on/off

#endif
