#ifndef __RS485_CRC_H
#define __RS485_CRC_H

#include <stdint.h>
#include <stdbool.h>

// CRC16-CCITT多项式
#define CRC16_CCITT_POLY  0x1021

// CRC表结构
typedef struct {
    uint16_t table[256];
    bool initialized;
} CRC16_Table_t;

// 初始化CRC表
void CRC16_InitTable(CRC16_Table_t* crc_table);

// 计算CRC16
uint16_t CRC16_Calculate(const uint8_t* data, uint16_t length);

// 使用预计算表计算CRC16
uint16_t CRC16_CalculateWithTable(CRC16_Table_t* crc_table, const uint8_t* data, uint16_t length);

// 验证CRC16
bool CRC16_Verify(const uint8_t* data, uint16_t length, uint16_t expected_crc);

// 在数据后追加CRC
void CRC16_Append(uint8_t* buffer, uint16_t data_length);

// 从缓冲区提取CRC（小端序）
uint16_t CRC16_Extract(const uint8_t* buffer, uint16_t offset);

#endif /* __RS485_CRC_H */