#include "oled_debug.h"
#include "../../Inc/u8g2/u8g2.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// U8g2对象
static u8g2_t u8g2;

// 显示缓冲区（8行文本）
static char display_buffer[8][32];
static uint8_t current_line = 0;

// 引脚定义
#define OLED_SCK_GPIO_Port  GPIOD
#define OLED_SCK_Pin        GPIO_PIN_14  // SCL
#define OLED_MOSI_GPIO_Port GPIOD
#define OLED_MOSI_Pin       GPIO_PIN_13  // SDA
#define OLED_DC_GPIO_Port   GPIOD
#define OLED_DC_Pin         GPIO_PIN_11  // DC
#define OLED_RES_GPIO_Port  GPIOD
#define OLED_RES_Pin        GPIO_PIN_12  // RES

// 软件SPI发送字节
static void OLED_SPI_SendByte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        // 设置数据线
        if (byte & 0x80) {
            HAL_GPIO_WritePin(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin, GPIO_PIN_RESET);
        }

        // 时钟上升沿
        HAL_GPIO_WritePin(OLED_SCK_GPIO_Port, OLED_SCK_Pin, GPIO_PIN_SET);
        for(volatile int j = 0; j < 10; j++);  // 短延时

        // 时钟下降沿
        HAL_GPIO_WritePin(OLED_SCK_GPIO_Port, OLED_SCK_Pin, GPIO_PIN_RESET);

        byte <<= 1;
    }
}

// GPIO和延时回调函数
static uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;

        case U8X8_MSG_DELAY_MILLI:
            HAL_Delay(arg_int);
            break;

        case U8X8_MSG_DELAY_10MICRO:
            for(volatile int i = 0; i < 320; i++);
            break;

        case U8X8_MSG_DELAY_100NANO:
            __NOP();
            break;

        case U8X8_MSG_GPIO_DC:
            HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        case U8X8_MSG_GPIO_RESET:
            HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        case U8X8_MSG_GPIO_SPI_CLOCK:
            HAL_GPIO_WritePin(OLED_SCK_GPIO_Port, OLED_SCK_Pin, arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        case U8X8_MSG_GPIO_SPI_DATA:
            HAL_GPIO_WritePin(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin, arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        default:
            return 0;
    }
    return 1;
}

// SPI字节发送回调
static uint8_t u8x8_byte_stm32_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    uint8_t *data;

    switch (msg) {
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            while (arg_int > 0) {
                OLED_SPI_SendByte(*data);
                data++;
                arg_int--;
            }
            break;

        case U8X8_MSG_BYTE_INIT:
            HAL_GPIO_WritePin(OLED_SCK_GPIO_Port, OLED_SCK_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin, GPIO_PIN_RESET);
            break;

        case U8X8_MSG_BYTE_SET_DC:
            HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            break;

        case U8X8_MSG_BYTE_END_TRANSFER:
            break;

        default:
            return 0;
    }
    return 1;
}

// 初始化OLED调试显示
void OLED_Debug_Init(void) {
    // 清空显示缓冲区
    memset(display_buffer, 0, sizeof(display_buffer));
    current_line = 0;
    
    // 硬件复位
    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    // 初始化U8g2 - 使用SSD1306 128x64
    u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_stm32_spi, u8x8_gpio_and_delay_stm32);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);  // 唤醒显示
    u8g2_ClearBuffer(&u8g2);
    
    // 设置字体（小字体，可以显示更多行）
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    
    // 显示启动信息
    OLED_Debug_Line(0, "OLED Debug Init");
    OLED_Debug_Update();
}

// 在指定行显示文本
void OLED_Debug_Line(uint8_t line, const char* format, ...) {
    if (line >= 8) return;
    
    va_list args;
    va_start(args, format);
    vsnprintf(display_buffer[line], sizeof(display_buffer[line]), format, args);
    va_end(args);
}

// 滚动显示调试信息
void OLED_Debug_Printf(const char* format, ...) {
    char temp[32];
    va_list args;
    va_start(args, format);
    vsnprintf(temp, sizeof(temp), format, args);
    va_end(args);
    
    // 滚动显示：把新内容加到底部
    if (current_line < 8) {
        strcpy(display_buffer[current_line], temp);
        current_line++;
    } else {
        // 向上滚动
        for (int i = 0; i < 7; i++) {
            strcpy(display_buffer[i], display_buffer[i + 1]);
        }
        strcpy(display_buffer[7], temp);
    }
    
    OLED_Debug_Update();
}

// 清屏
void OLED_Debug_Clear(void) {
    memset(display_buffer, 0, sizeof(display_buffer));
    current_line = 0;
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
}

// 显示CAN调试信息
void OLED_Debug_ShowCANInfo(uint32_t can_id, uint8_t* data, uint8_t len) {
    char line1[32], line2[32];
    
    // 第一行：CAN ID和消息类型
    if ((can_id >= 0x700) && (can_id <= 0x77F)) {
        snprintf(line1, sizeof(line1), "HB ID:%03lX Node:%lu", (unsigned long)can_id, (unsigned long)(can_id - 0x700));
    } else if ((can_id >= 0x580) && (can_id <= 0x5FF)) {
        snprintf(line1, sizeof(line1), "SDO ID:%03lX", (unsigned long)can_id);
    } else {
        snprintf(line1, sizeof(line1), "CAN ID:%03lX", (unsigned long)can_id);
    }
    
    // 第二行：数据
    if (len > 4) {
        snprintf(line2, sizeof(line2), "D:%02X %02X %02X %02X...", 
                 data[0], data[1], data[2], data[3]);
    } else {
        line2[0] = 'D';
        line2[1] = ':';
        int pos = 2;
        for (int i = 0; i < len && i < 8; i++) {
            pos += snprintf(line2 + pos, sizeof(line2) - pos, "%02X ", data[i]);
        }
    }
    
    OLED_Debug_Printf(line1);
    OLED_Debug_Printf(line2);
}

// 显示系统状态
void OLED_Debug_ShowStatus(const char* status) {
    // 在第一行显示状态
    OLED_Debug_Line(0, "Status: %s", status);
    OLED_Debug_Update();
}

// 更新显示
void OLED_Debug_Update(void) {
    u8g2_ClearBuffer(&u8g2);
    
    // 显示所有行
    for (int i = 0; i < 8; i++) {
        if (display_buffer[i][0] != 0) {
            u8g2_DrawStr(&u8g2, 0, (i + 1) * 8, display_buffer[i]);
        }
    }
    
    u8g2_SendBuffer(&u8g2);
}