/**
 * RS485通信测试程序 - Master MCU
 * 用于调试RS485通信问题
 */

#include "rs485_test.h"
#include "rs485.h"
/* 先包含blackboard.h，避免宏定义冲突 */
#include "../Blackboard/blackboard.h"
#include "rs485_protocol.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* 外部UART句柄 */
extern UART_HandleTypeDef huart5;

/* 测试状态 */
static struct {
    uint32_t test_count;
    uint32_t send_success;
    uint32_t send_fail;
    uint32_t recv_count;
    uint32_t last_test_time;
    uint8_t test_mode;  // 0=停止, 1=简单测试, 2=协议测试, 3=回环测试
} test_state = {0};

/**
 * @brief 初始化RS485测试
 */
void RS485_Test_Init(void) {
    memset(&test_state, 0, sizeof(test_state));
    test_state.test_mode = 1;  // 默认简单测试模式
    
    printf("\r\n=== RS485 TEST MODULE INITIALIZED ===\r\n");
    printf("UART5 Configuration:\r\n");
    printf("  Baudrate: 115200\r\n");
    printf("  WordLength: 8 bits\r\n");
    printf("  StopBits: 1\r\n");
    printf("  Parity: None\r\n");
    printf("  Mode: TX_RX\r\n");
    printf("\r\n");
}

/**
 * @brief 发送简单测试数据包
 */
void RS485_Test_SendSimple(void) {
    uint8_t test_data[8] = {0xAA, 0x55, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    
    printf("[RS485_TEST] Sending simple test packet\r\n");
    printf("  Data: ");
    for(int i = 0; i < 8; i++) {
        printf("0x%02X ", test_data[i]);
    }
    printf("\r\n");
    
    /* 方法1: 使用HAL_UART_Transmit (阻塞式) */
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart5, test_data, 8, 100);
    
    if (status == HAL_OK) {
        test_state.send_success++;
        printf("  [OK] HAL_UART_Transmit success\r\n");
    } else {
        test_state.send_fail++;
        printf("  [FAIL] HAL_UART_Transmit error: %d\r\n", status);
    }
    
    /* 方法2: 尝试DMA发送 */
    rs485_status_t rs_status = rs485_send_packet_dma(test_data, 8);
    if (rs_status == RS485_OK) {
        printf("  [OK] DMA send initiated\r\n");
    } else if (rs_status == RS485_BUSY) {
        printf("  [BUSY] DMA is busy\r\n");
    } else {
        printf("  [FAIL] DMA send error\r\n");
    }
    
    test_state.test_count++;
}

/**
 * @brief 发送协议测试数据包（模拟内呼）
 */
void RS485_Test_SendProtocol(void) {
    uint8_t cabin_call[4] = {CMD_CABIN_CALL, 2, 0, 0};  // 2楼内呼
    
    printf("[RS485_TEST] Sending protocol test (Cabin Call F2)\r\n");
    printf("  CMD: 0x%02X (CABIN_CALL)\r\n", cabin_call[0]);
    printf("  Floor: %d\r\n", cabin_call[1]);
    
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart5, cabin_call, 4, 100);
    
    if (status == HAL_OK) {
        test_state.send_success++;
        printf("  [OK] Protocol packet sent\r\n");
    } else {
        test_state.send_fail++;
        printf("  [FAIL] Send error: %d\r\n", status);
    }
}

/**
 * @brief 接收测试 - 检查接收缓冲区
 */
void RS485_Test_CheckReceive(void) {
    /* 方法1: 使用RS485驱动接收 */
    uint8_t rx_buffer[64];
    uint16_t rx_len = rs485_receive_packet(rx_buffer, sizeof(rx_buffer));
    
    if (rx_len > 0) {
        test_state.recv_count++;
        printf("\r\n[RX] %d bytes: ", rx_len);
        for(int i = 0; i < rx_len && i < 16; i++) {
            printf("0x%02X ", rx_buffer[i]);
        }
        
        /* 解析协议 */
        if (rx_len >= 2) {
            switch(rx_buffer[0]) {
                case CMD_CABIN_CALL:
                    printf("\r\n  => Cabin Call Floor %d", rx_buffer[1]);
                    /* 将内呼加入黑板 */
                    Blackboard_AddCabinCall(rx_buffer[1]);
                    printf(" [Added to BB]");
                    break;
                case CMD_PHOTO_SENSOR:
                    printf("\r\n  => Photo Sensor Floor %d", rx_buffer[1]);
                    break;
                case CMD_DOOR_STATUS:
                    printf("\r\n  => Door Status: %s", rx_buffer[1] ? "OPEN" : "CLOSED");
                    break;
                case CMD_STATUS_RESPONSE:
                    printf("\r\n  => Status: F%d, Dir=%d, Door=%d",
                           rx_buffer[1], rx_buffer[2], rx_buffer[3]);
                    break;
                default:
                    printf("\r\n  => CMD: 0x%02X", rx_buffer[0]);
            }
        }
        printf("\r\n");
    }
    
    /* 方法2: 直接尝试HAL接收（非阻塞） */
    uint8_t direct_rx[4];
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart5, direct_rx, 1, 1);
    if (status == HAL_OK) {
        printf("[HAL_RX] 0x%02X", direct_rx[0]);
        
        /* 尝试读取更多 */
        status = HAL_UART_Receive(&huart5, &direct_rx[1], 3, 10);
        if (status == HAL_OK) {
            printf(" 0x%02X 0x%02X 0x%02X\r\n",
                   direct_rx[1], direct_rx[2], direct_rx[3]);
        } else {
            printf("\r\n");
        }
    }
}

/**
 * @brief 回环测试 - 发送并等待回显
 */
void RS485_Test_Loopback(void) {
    static uint8_t sequence = 0;
    uint8_t loopback_data[4] = {0xEE, sequence++, 0x00, 0xFF};
    
    printf("[RS485_TEST] Loopback test, seq=%d\r\n", loopback_data[1]);
    
    /* 清空接收缓冲区 */
    rs485_clear_rx_buffer();
    
    /* 发送数据 */
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart5, loopback_data, 4, 100);
    if (status != HAL_OK) {
        printf("  [FAIL] Send error\r\n");
        return;
    }
    
    /* 等待回显 */
    HAL_Delay(10);
    
    /* 检查接收 */
    uint8_t rx_buffer[16];
    uint16_t rx_len = rs485_receive_packet(rx_buffer, sizeof(rx_buffer));
    
    if (rx_len > 0) {
        printf("  [LOOPBACK] Received %d bytes: ", rx_len);
        for(int i = 0; i < rx_len; i++) {
            printf("0x%02X ", rx_buffer[i]);
        }
        
        /* 验证数据 */
        if (rx_len == 4 && memcmp(loopback_data, rx_buffer, 4) == 0) {
            printf(" => MATCH!\r\n");
        } else {
            printf(" => MISMATCH!\r\n");
        }
    } else {
        printf("  [LOOPBACK] No echo received\r\n");
    }
}

/**
 * @brief RS485测试主处理函数
 */
void RS485_Test_Process(void) {
    uint32_t current_time = HAL_GetTick();
    
    /* 每2秒执行一次测试 */
    if (current_time - test_state.last_test_time < 2000) {
        return;
    }
    test_state.last_test_time = current_time;
    
    /* 先检查接收 */
    RS485_Test_CheckReceive();
    
    /* 根据测试模式执行不同测试 */
    switch(test_state.test_mode) {
        case 1:  // 简单测试
            RS485_Test_SendSimple();
            break;
            
        case 2:  // 协议测试
            RS485_Test_SendProtocol();
            break;
            
        case 3:  // 回环测试
            RS485_Test_Loopback();
            break;
            
        default:
            break;
    }
    
    /* 每10次打印统计 */
    if (test_state.test_count % 10 == 0 && test_state.test_count > 0) {
        RS485_Test_PrintStats();
    }
}

/**
 * @brief 设置测试模式
 */
void RS485_Test_SetMode(uint8_t mode) {
    test_state.test_mode = mode;
    printf("[RS485_TEST] Mode set to %d\r\n", mode);
}

/**
 * @brief 打印测试统计
 */
void RS485_Test_PrintStats(void) {
    printf("\r\n=== RS485 TEST STATISTICS ===\r\n");
    printf("Test Count: %lu\r\n", test_state.test_count);
    printf("Send Success: %lu\r\n", test_state.send_success);
    printf("Send Fail: %lu\r\n", test_state.send_fail);
    printf("Receive Count: %lu\r\n", test_state.recv_count);
    
    /* 获取RS485驱动统计 */
    rs485_stats_t stats;
    rs485_get_stats(&stats);
    printf("\r\nDriver Stats:\r\n");
    printf("  TX Packets: %lu\r\n", stats.tx_packets);
    printf("  TX Bytes: %lu\r\n", stats.tx_bytes);
    printf("  TX Errors: %lu\r\n", stats.tx_errors);
    printf("  RX Packets: %lu\r\n", stats.rx_packets);
    printf("  RX Bytes: %lu\r\n", stats.rx_bytes);
    printf("  RX Errors: %lu\r\n", stats.rx_errors);
    printf("=============================\r\n\r\n");
}

/**
 * @brief 手动发送测试命令
 */
void RS485_Test_SendCommand(uint8_t cmd, uint8_t param1, uint8_t param2, uint8_t param3) {
    uint8_t data[4] = {cmd, param1, param2, param3};
    
    printf("[RS485_TEST] Manual command: ");
    printf("CMD=0x%02X P1=%d P2=%d P3=%d\r\n", cmd, param1, param2, param3);
    
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart5, data, 4, 100);
    
    if (status == HAL_OK) {
        printf("  [OK] Sent successfully\r\n");
    } else {
        printf("  [FAIL] Send error: %d\r\n", status);
    }
}