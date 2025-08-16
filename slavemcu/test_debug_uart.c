/**
  ******************************************************************************
  * @file    test_debug_uart.c
  * @brief   Test program for USART1 debug output
  ******************************************************************************
  * Hardware connections:
  * 
  * STM32F103C8 -> USB-to-TTL adapter
  * PA9  (TX) -> RXD
  * PA10 (RX) -> TXD
  * GND       -> GND
  * 
  * PC Terminal settings:
  * - Baud rate: 115200
  * - Data bits: 8
  * - Stop bits: 1
  * - Parity: None
  * - Flow control: None
  * 
  * Software: PuTTY, Tera Term, or Arduino Serial Monitor
  ******************************************************************************
  */

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "../Modules/LED/LED.h"
#include <stdio.h>

void Test_Debug_UART(void)
{
    static uint32_t counter = 0;
    static uint32_t last_print_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Print debug message every second
    if (current_time - last_print_time >= 1000) {
        last_print_time = current_time;
        counter++;
        
        // Send debug message to USART1
        printf("[%lu] Debug UART Test - Counter: %lu\r\n", current_time, counter);
        printf("  System running normally\r\n");
        printf("  LED toggle...\r\n");
        
        // Toggle LED to show activity
        LED_Toggle();
    }
}

// Alternative simple test - call this in main loop
void Simple_UART_Test(void)
{
    // Send a simple message
    printf("Hello from STM32F103C8!\r\n");
    printf("USART1 Debug Output Working!\r\n");
    printf("PA9=TX, PA10=RX, 115200 baud\r\n");
    printf("================================\r\n");
    
    // Flash LED to indicate message sent
    LED_Flash(1);
}