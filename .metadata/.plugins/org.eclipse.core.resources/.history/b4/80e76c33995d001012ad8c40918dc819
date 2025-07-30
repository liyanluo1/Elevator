#ifndef __LRF_MODULE_H
#define __LRF_MODULE_H

#include "../../Core/Inc/main.h"

/* === 外部宏 === */
#define LRF_UART               huart3          // ← 和 usart.c 里的句柄同名
#define LRF_PWR_GPIO_Port      GPIOA
#define LRF_PWR_Pin            GPIO_PIN_8

/* === API === */
void  LRF_Init(void);               // 上电 + 波特率同步
int   LRF_ReadDistance_cm(uint32_t *distance_cm); // 读一次距离，成功返回0

#endif /* __LRF_MODULE_H */

