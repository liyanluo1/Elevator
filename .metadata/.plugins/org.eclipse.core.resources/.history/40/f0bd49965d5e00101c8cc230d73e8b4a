#ifndef DISPLAY_H
#define DISPLAY_H

#include "stm32f4xx_hal.h"
#include "../../Inc/u8g2/u8g2.h"
#include "../Stepper/motor_control.h"
#include <stdio.h>

// 显示模块初始化
void Display_Init(void);

// 显示电机状态
void Display_MotorStatus(Motor_t* motor);

// 显示自定义文本
void Display_Text(uint8_t x, uint8_t y, const char* text);

// 清屏
void Display_Clear(void);

// 发送缓冲区到屏幕
void Display_Update(void);

// 显示多行信息
void Display_MultiLine(const char* line1, const char* line2, const char* line3, const char* line4);

// 显示欢迎界面
void Display_Welcome(void);

// 显示系统信息
void Display_SystemInfo(void);

// 显示错误信息
void Display_Error(const char* error_msg);

#endif /* DISPLAY_H */
