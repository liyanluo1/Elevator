#ifndef __RS485_COMM_H
#define __RS485_COMM_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// RS485�6
#define RS485_DE_GPIO_Port  GPIOB
#define RS485_DE_Pin        GPIO_PIN_0

// ��p
#define RS485_BAUDRATE      9600
#define RS485_TIMEOUT       100     // ��

// '<�I
#define FRAME_START         0xAA
#define FRAME_END           0x55
#define MAX_DATA_LENGTH     16

// }�{�
typedef enum {
    CMD_DOOR_OPEN = 0x01,
    CMD_DOOR_CLOSE = 0x02,
    CMD_DOOR_STATUS = 0x03,
    CMD_SENSOR_STATUS = 0x04,
    CMD_KEYPAD_DATA = 0x05,
    CMD_HEARTBEAT = 0x06,
    CMD_ACK = 0x07,
    CMD_NACK = 0x08
} CommandType_t;

// �6}�
typedef enum {
    DOOR_OPEN = 0x01,
    DOOR_CLOSE = 0x02,
    DOOR_STOP = 0x03
} DoorCommand_t;

// pn'ӄ
typedef struct {
    uint8_t start;          // w�W�
    uint8_t length;         // pn�
    uint8_t cmd;            // }�{�
    uint8_t data[MAX_DATA_LENGTH];  // pn
    uint8_t checksum;       // !��
    uint8_t end;            // �_W�
} RS485_Frame_t;

// �6�:�
typedef enum {
    RX_STATE_IDLE = 0,
    RX_STATE_LENGTH,
    RX_STATE_CMD,
    RX_STATE_DATA,
    RX_STATE_CHECKSUM,
    RX_STATE_END
} RxState_t;

// �6�:
typedef struct {
    RS485_Frame_t frame;
    uint8_t data_index;
    RxState_t state;
    uint32_t last_rx_time;
} RxBuffer_t;

// �p�
void RS485_Init(void);
void RS485_Process(void);
bool RS485_SendFrame(CommandType_t cmd, uint8_t* data, uint8_t length);
bool RS485_SendDoorCommand(DoorCommand_t door_cmd);
bool RS485_RequestSensorStatus(void);
bool RS485_SendHeartbeat(void);

// ���p
void RS485_SetMode(bool transmit);
uint8_t RS485_CalculateChecksum(uint8_t* data, uint8_t length);
void RS485_ProcessRxFrame(RS485_Frame_t* frame);
void RS485_UART_RxCallback(uint8_t data);

// ���
bool RS485_IsConnected(void);
uint32_t RS485_GetLastRxTime(void);

#endif /* __RS485_COMM_H */