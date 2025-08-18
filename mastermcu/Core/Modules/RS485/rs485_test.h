#ifndef __RS485_TEST_H
#define __RS485_TEST_H

#include <stdint.h>

/* 测试模式定义 */
#define TEST_MODE_STOP      0
#define TEST_MODE_SIMPLE    1
#define TEST_MODE_PROTOCOL  2
#define TEST_MODE_LOOPBACK  3

/* 函数声明 */
void RS485_Test_Init(void);
void RS485_Test_Process(void);
void RS485_Test_SetMode(uint8_t mode);
void RS485_Test_PrintStats(void);
void RS485_Test_SendCommand(uint8_t cmd, uint8_t param1, uint8_t param2, uint8_t param3);

/* 单独的测试函数 */
void RS485_Test_SendSimple(void);
void RS485_Test_SendProtocol(void);
void RS485_Test_CheckReceive(void);
void RS485_Test_Loopback(void);

#endif /* __RS485_TEST_H */