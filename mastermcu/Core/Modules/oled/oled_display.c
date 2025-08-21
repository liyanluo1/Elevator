#include "oled_display.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>

/* 外部U8g2对象 */
extern u8g2_t u8g2;

/* 内部变量 */
static uint32_t last_update_time = 0;
static uint8_t last_floor = 0;
static bool last_cabin[4] = {0};
static bool last_up[4] = {0};
static bool last_down[4] = {0};

/* 初始化OLED显示 */
void OLED_Display_Init(void) {
    /* 初始化底层OLED驱动 */
    Display_Init();
    
    /* 显示欢迎界面 */
    OLED_Display_Welcome();
    HAL_Delay(1000);
    
    /* 清屏准备显示 */
    OLED_Display_Clear();
    
    printf("[OLED] Display initialized\r\n");
}

/* 显示欢迎界面 */
void OLED_Display_Welcome(void) {
    u8g2_ClearBuffer(&u8g2);
    
    /* 设置字体 */
    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
    
    /* 显示欢迎文字 */
    u8g2_DrawStr(&u8g2, 20, 25, "ELEVATOR");
    
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
    u8g2_DrawStr(&u8g2, 25, 45, "SYSTEM");
    u8g2_DrawStr(&u8g2, 35, 60, "V2.0");
    
    u8g2_SendBuffer(&u8g2);
}

/* 更新显示内容 */
void OLED_Display_Update(uint8_t current_floor, 
                         bool cabin_calls[4],
                         bool up_calls[4], 
                         bool down_calls[4]) {
    
    /* 限制更新频率 */
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_update_time < OLED_UPDATE_INTERVAL_MS) {
        return;
    }
    last_update_time = current_time;
    
    /* 检查是否有变化 */
    bool changed = false;
    if (current_floor != last_floor) {
        changed = true;
        last_floor = current_floor;
    }
    
    for (int i = 1; i <= 3; i++) {
        if (cabin_calls[i] != last_cabin[i] ||
            up_calls[i] != last_up[i] ||
            down_calls[i] != last_down[i]) {
            changed = true;
            last_cabin[i] = cabin_calls[i];
            last_up[i] = up_calls[i];
            last_down[i] = down_calls[i];
        }
    }
    
    /* 如果没有变化，不更新显示 */
    if (!changed) {
        return;
    }
    
    /* 清除缓冲区 */
    u8g2_ClearBuffer(&u8g2);
    
    /* ===== 1. 显示当前楼层（大字体） ===== */
    u8g2_SetFont(&u8g2, u8g2_font_logisoso24_tn);  // 大数字字体
    char floor_str[4];
    sprintf(floor_str, "%d", current_floor);
    
    /* 计算居中位置 */
    int floor_width = u8g2_GetStrWidth(&u8g2, floor_str);
    int floor_x = (128 - floor_width - 10) / 2;  // 减10是为了给F留空间
    
    /* 显示楼层数字 */
    u8g2_DrawStr(&u8g2, floor_x, 24, floor_str);
    
    /* 显示F（使用中等字体） */
    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&u8g2, floor_x + floor_width + 2, 24, "F");
    
    /* 画分割线 */
    u8g2_DrawHLine(&u8g2, 0, 27, 128);
    
    /* ===== 2. 显示内呼 ===== */
    u8g2_SetFont(&u8g2, u8g2_font_7x14_tf);  // 中文支持字体
    u8g2_DrawStr(&u8g2, 2, 40, "Cabin:");
    
    /* 显示各楼层内呼状态 */
    for (int i = 1; i <= 3; i++) {
        int x = 50 + (i - 1) * 25;
        char num_str[3];
        sprintf(num_str, "%d", i);
        
        if (cabin_calls[i]) {
            /* 有呼叫 - 画实心方框 */
            u8g2_DrawBox(&u8g2, x - 2, 30, 16, 14);
            u8g2_SetDrawColor(&u8g2, 0);  // 反色
            u8g2_DrawStr(&u8g2, x + 3, 40, num_str);
            u8g2_SetDrawColor(&u8g2, 1);  // 恢复正常
        } else {
            /* 无呼叫 - 画空心方框 */
            u8g2_DrawFrame(&u8g2, x - 2, 30, 16, 14);
            u8g2_DrawStr(&u8g2, x + 3, 40, num_str);
        }
    }
    
    /* 画分割线 */
    u8g2_DrawHLine(&u8g2, 0, 45, 128);
    
    /* ===== 3. 显示外呼 ===== */
    u8g2_SetFont(&u8g2, u8g2_font_7x14_tf);
    u8g2_DrawStr(&u8g2, 2, 58, "Hall:");
    
    int x_pos = 45;
    
    /* 显示上行呼叫 */
    for (int i = 1; i <= 2; i++) {  // 1楼和2楼有上行
        if (up_calls[i]) {
            /* 画上三角形 */
            u8g2_DrawTriangle(&u8g2, 
                x_pos + 4, 48,      // 顶点
                x_pos, 56,          // 左下
                x_pos + 8, 56);     // 右下
            
            /* 显示楼层号 */
            char num[2];
            sprintf(num, "%d", i);
            u8g2_DrawStr(&u8g2, x_pos + 10, 58, num);
            x_pos += 20;
        }
    }
    
    /* 显示下行呼叫 */
    for (int i = 2; i <= 3; i++) {  // 2楼和3楼有下行
        if (down_calls[i]) {
            /* 画下三角形 */
            u8g2_DrawTriangle(&u8g2,
                x_pos + 4, 56,      // 底点
                x_pos, 48,          // 左上
                x_pos + 8, 48);     // 右上
            
            /* 显示楼层号 */
            char num[2];
            sprintf(num, "%d", i);
            u8g2_DrawStr(&u8g2, x_pos + 10, 58, num);
            x_pos += 20;
        }
    }
    
    /* 如果没有任何外呼，显示提示 */
    if (x_pos == 45) {
        u8g2_DrawStr(&u8g2, 45, 58, "None");
    }
    
    /* 发送缓冲区到屏幕 */
    u8g2_SendBuffer(&u8g2);
}

/* 显示错误信息 */
void OLED_Display_Error(const char* msg) {
    u8g2_ClearBuffer(&u8g2);
    
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
    u8g2_DrawStr(&u8g2, 30, 25, "ERROR!");
    
    u8g2_SetFont(&u8g2, u8g2_font_7x14_tf);
    
    /* 自动换行显示错误信息 */
    int len = strlen(msg);
    char line[20];
    int y = 45;
    int start = 0;
    
    while (start < len && y < 64) {
        int chars = (len - start > 18) ? 18 : (len - start);
        strncpy(line, msg + start, chars);
        line[chars] = '\0';
        u8g2_DrawStr(&u8g2, 2, y, line);
        start += chars;
        y += 12;
    }
    
    u8g2_SendBuffer(&u8g2);
}

/* 清屏 */
void OLED_Display_Clear(void) {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
}