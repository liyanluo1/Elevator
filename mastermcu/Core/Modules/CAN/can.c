#include "can.h"

/* extern 句柄由 main.c/ CubeMX 提供 */
extern CAN_HandleTypeDef hcan1;

/* 错误处理函数 */
extern void Error_Handler(void);

/* 启动 CAN + 过滤器：全接收到 FIFO0 */
void CAN1_UserFilterStart(void)
{
    CAN_FilterTypeDef f = {0};
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;  // 接收所有ID
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;  // 掩码全为0 = 接收所有
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    
    // 配置过滤器
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) {
        Error_Handler();
    }

    // 启动CAN - 这一步很重要！
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    
    // 使能RX FIFO0消息挂起中断
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

/* 发送可变长度CAN数据帧 */
uint8_t CAN1_Send_Frame(uint16_t std_id, const uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox;
    
    // 参数检查
    if (length > 8) {
        return 1;  // CAN最大只支持8字节
    }

    tx.StdId  = std_id & 0x7FF;  // 11位标准ID
    tx.IDE    = CAN_ID_STD;      // 标准帧
    tx.RTR    = CAN_RTR_DATA;    // 数据帧
    tx.DLC    = length;          // 数据长度

    return (HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t*)data, &mailbox) == HAL_OK) ? 0 : 1;
}

/* 兼容旧接口 - 固定8字节 (CANopen需要) */
uint8_t CAN1_Send_Num(uint16_t std_id, const uint8_t *data)
{
    return CAN1_Send_Frame(std_id, data, 8);
}

/* CAN接收回调函数指针 */
static CAN_RxCallback_t g_can_rx_callback = NULL;

/* 注册CAN接收回调函数 */
void CAN1_RegisterRxCallback(CAN_RxCallback_t callback) {
    g_can_rx_callback = callback;
}

/* CAN接收中断回调 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (hcan->Instance == CAN1) {
        // 从FIFO0获取消息
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            // 调用注册的回调函数
            if (g_can_rx_callback != NULL) {
                g_can_rx_callback(rx_header.StdId, rx_data);
            }
        }
    }
}

/* 用户自定义CAN接收处理 */
void CAN1_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 用户可以重写此函数来处理接收到的CAN消息
    // 默认调用HAL库的回调
    HAL_CAN_RxFifo0MsgPendingCallback(hcan);
}