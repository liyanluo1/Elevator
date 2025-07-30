#include "laser_sensor.h"
#include "string.h"
#include "usart.h"
/* 私有命令帧示例 —— 以常见 0xAA 开头协议为例 */
static const uint8_t cmd_read_dist[] = {0xAA, 0x04, 0x00, 0x00}; // 简化：读距离

/* ---- 上电与同步 -------------------------------------------------*/
void LRF_Init(void)
{
    /* 1) 给模块上电 */
    HAL_GPIO_WritePin(LRF_PWR_GPIO_Port, LRF_PWR_Pin, GPIO_PIN_SET);
    HAL_Delay(100);                       // 自检

    /* 2) 发送 0x55 做自动波特率同步 */
    uint8_t sync = 0x55;
    HAL_UART_Transmit(&LRF_UART, &sync, 1, 10);
    HAL_Delay(10);
}


/* ---- 读取一次距离 ----------------------------------------------*/
int LRF_ReadDistance_cm(uint32_t *distance_cm)
{
    uint8_t rx[8] = {0};

    /* 1) 发送读距离命令 */
    HAL_UART_Transmit(&LRF_UART,
                      (uint8_t*)cmd_read_dist,
                      sizeof(cmd_read_dist),
                      HAL_MAX_DELAY);

    /* 2) 接收返回帧（假设 8 字节） */
    if (HAL_OK != HAL_UART_Receive(&LRF_UART, rx, 8, 100))
        return -1;                        // 超时

    /* 3) 校验帧头+帧尾，可按手册做 CRC/校验和 */
    if (rx[0] != 0xAA)
        return -2;

    /* 4) 解析距离，高字节在前 (示例) */
    *distance_cm = (rx[2] << 8) | rx[3];
    return 0;
}
