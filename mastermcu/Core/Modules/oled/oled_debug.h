#ifndef OLED_DEBUG_H
#define OLED_DEBUG_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// OLED调试显示初始化
void OLED_Debug_Init(void);

// 显示调试信息（最多5行）
void OLED_Debug_Printf(const char* format, ...);

// 清屏
void OLED_Debug_Clear(void);

// 显示CAN调试信息
void OLED_Debug_ShowCANInfo(uint32_t can_id, uint8_t* data, uint8_t len);

// 显示系统状态
void OLED_Debug_ShowStatus(const char* status);

// 更新显示
void OLED_Debug_Update(void);

// 在指定行显示文本（行号0-7）
void OLED_Debug_Line(uint8_t line, const char* format, ...);

#endif /* OLED_DEBUG_H */