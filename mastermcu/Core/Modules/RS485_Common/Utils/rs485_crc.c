#include "rs485_crc.h"

// 静态CRC表（可选）
static CRC16_Table_t g_crc16_table = {.initialized = false};

// 初始化CRC表
void CRC16_InitTable(CRC16_Table_t* crc_table) {
    if (crc_table == NULL || crc_table->initialized) {
        return;
    }
    
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_CCITT_POLY;
            } else {
                crc <<= 1;
            }
        }
        crc_table->table[i] = crc;
    }
    
    crc_table->initialized = true;
}

// 计算CRC16（直接计算）
uint16_t CRC16_Calculate(const uint8_t* data, uint16_t length) {
    if (data == NULL || length == 0) {
        return 0;
    }
    
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_CCITT_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

// 使用预计算表计算CRC16
uint16_t CRC16_CalculateWithTable(CRC16_Table_t* crc_table, const uint8_t* data, uint16_t length) {
    if (crc_table == NULL || data == NULL || length == 0) {
        return 0;
    }
    
    // 如果表未初始化，则初始化
    if (!crc_table->initialized) {
        CRC16_InitTable(crc_table);
    }
    
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        uint8_t index = (crc >> 8) ^ data[i];
        crc = (crc << 8) ^ crc_table->table[index];
    }
    
    return crc;
}

// 验证CRC16
bool CRC16_Verify(const uint8_t* data, uint16_t length, uint16_t expected_crc) {
    if (data == NULL || length == 0) {
        return false;
    }
    
    uint16_t calculated_crc = CRC16_Calculate(data, length);
    return (calculated_crc == expected_crc);
}

// 在数据后追加CRC（小端序）
void CRC16_Append(uint8_t* buffer, uint16_t data_length) {
    if (buffer == NULL || data_length == 0) {
        return;
    }
    
    uint16_t crc = CRC16_Calculate(buffer, data_length);
    
    // 小端序存储
    buffer[data_length] = (uint8_t)(crc & 0xFF);
    buffer[data_length + 1] = (uint8_t)((crc >> 8) & 0xFF);
}

// 从缓冲区提取CRC（小端序）
uint16_t CRC16_Extract(const uint8_t* buffer, uint16_t offset) {
    if (buffer == NULL) {
        return 0;
    }
    
    return (uint16_t)(buffer[offset] | (buffer[offset + 1] << 8));
}

// 获取全局CRC表（单例模式）
CRC16_Table_t* CRC16_GetGlobalTable(void) {
    if (!g_crc16_table.initialized) {
        CRC16_InitTable(&g_crc16_table);
    }
    return &g_crc16_table;
}