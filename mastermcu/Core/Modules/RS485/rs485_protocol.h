#ifndef __RS485_PROTOCOL_H__
#define __RS485_PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

/* ==================== RS485 Protocol Commands ==================== */

/* 命令定义 - 使用宏定义方便两个MCU分别使用 */
#define CMD_PHOTO_SENSOR    0x10  /* 光电传感器触发: [CMD][Floor][0][0] */
#define CMD_DOOR_OPEN       0x20  /* 开门命令: [CMD][0][0][0] */
#define CMD_DOOR_CLOSE      0x21  /* 关门命令: [CMD][0][0][0] */
#define CMD_DOOR_STATUS     0x22  /* 门状态: [CMD][Status][0][0] */
#define CMD_FLOOR_CALL      0x30  /* 楼层外呼: [CMD][Floor][Dir][0] */
#define CMD_CABIN_CALL      0x31  /* 轿厢内呼: [CMD][Floor][0][0] */
#define CMD_DIRECTION_SET   0x40  /* 设置方向: [CMD][Dir][CurFloor][TargetFloor] */
#define CMD_STATUS_REQUEST  0x50  /* 状态请求: [CMD][0][0][0] */
#define CMD_STATUS_RESPONSE 0x51  /* 状态响应: [CMD][Floor][Dir][Door] */
#define CMD_ERROR           0xF0  /* 错误报告: [CMD][ErrCode][0][0] */

/* 方向定义 */
#define DIR_STOP    0  /* 停止 */
#define DIR_UP      1  /* 上行 */
#define DIR_DOWN    2  /* 下行 */

/* 数据包结构 */
typedef struct {
    uint8_t cmd;        /* 命令类型 */
    uint8_t data[7];    /* 数据负载 */
    uint8_t length;     /* 数据长度 */
} rs485_packet_t;

/* 光电传感器数据 */
typedef struct {
    uint8_t floor;      /* 楼层号 1-3 */
    uint8_t reserved;
} photo_sensor_data_t;

/* 门控命令数据 */
typedef struct {
    uint8_t action;     /* 0=close, 1=open */
    uint8_t floor;      /* 楼层号 */
} door_control_data_t;

/* 协议处理回调 */
typedef struct {
    void (*photo_sensor_callback)(uint8_t floor);
    void (*door_command_callback)(uint8_t action, uint8_t floor);
    void (*floor_call_callback)(uint8_t floor, uint8_t direction);
    void (*status_request_callback)(void);
} rs485_callbacks_t;

/* 函数声明 */
void RS485_Protocol_Init(rs485_callbacks_t *callbacks);
void RS485_Protocol_ProcessPacket(uint8_t *data, uint16_t length);

/* 发送函数 */
bool RS485_Protocol_SendPhotoSensor(uint8_t floor);
bool RS485_Protocol_SendDoorCommand(uint8_t action, uint8_t floor);
bool RS485_Protocol_SendFloorCall(uint8_t floor, uint8_t direction);
bool RS485_Protocol_SendStatus(uint8_t current_floor, uint8_t door_status, uint8_t moving);
bool RS485_Protocol_SendError(uint8_t error_code);

#endif /* __RS485_PROTOCOL_H__ */