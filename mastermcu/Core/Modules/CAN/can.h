#ifndef __USER_CAN_H
#define __USER_CAN_H

#include "main.h"        /* 带 stm32f4xx_hal.h */

/* CAN句柄声明 */
extern CAN_HandleTypeDef hcan1;

/* CAN初始化和过滤器配置 */
void CAN1_UserFilterStart(void);

/* CAN发送函数 - 发送标准帧
 * @param std_id: 标准CAN ID (11位)
 * @param data: 数据指针
 * @param length: 数据长度 (0-8字节)
 * @return: 0成功, 1失败
 */
uint8_t CAN1_Send_Frame(uint16_t std_id, const uint8_t *data, uint8_t length);

/* 兼容旧接口 - 固定8字节 (CANopen兼容) */
uint8_t CAN1_Send_Num(uint16_t std_id, const uint8_t *data);

/* CAN接收回调处理 */
void CAN1_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
