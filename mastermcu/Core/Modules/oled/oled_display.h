#ifndef __OLED_DISPLAY_H
#define __OLED_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>

/* 显示更新间隔 */
#define OLED_UPDATE_INTERVAL_MS  100   // 100ms更新一次

/* 显示位置定义 */
#define FLOOR_DISPLAY_Y     8    // 楼层显示Y坐标
#define CABIN_DISPLAY_Y     32   // 内呼显示Y坐标  
#define HALL_DISPLAY_Y      48   // 外呼显示Y坐标

/* 初始化OLED显示 */
void OLED_Display_Init(void);

/* 更新显示内容 - 主循环调用 */
void OLED_Display_Update(uint8_t current_floor, 
                         bool cabin_calls[4],
                         bool up_calls[4], 
                         bool down_calls[4]);

/* 显示欢迎界面 */
void OLED_Display_Welcome(void);

/* 显示错误信息 */
void OLED_Display_Error(const char* msg);

/* 清屏 */
void OLED_Display_Clear(void);

#endif /* __OLED_DISPLAY_H */