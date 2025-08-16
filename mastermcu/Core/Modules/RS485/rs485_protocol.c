#include "rs485_protocol.h"
#include "rs485.h"
#include <string.h>
#include <stdio.h>

/* 私有变量 */
static rs485_callbacks_t protocol_callbacks;
static bool is_initialized = false;

/* 初始化协议层 */
void RS485_Protocol_Init(rs485_callbacks_t *callbacks)
{
    if (callbacks) {
        memcpy(&protocol_callbacks, callbacks, sizeof(rs485_callbacks_t));
        is_initialized = true;
    }
}

/* 处理接收到的数据包 */
void RS485_Protocol_ProcessPacket(uint8_t *data, uint16_t length)
{
    if (!is_initialized || length < 1) {
        return;
    }
    
    uint8_t cmd = data[0];
    
    switch (cmd) {
        case CMD_PHOTO_SENSOR:
            if (length >= 2 && protocol_callbacks.photo_sensor_callback) {
                uint8_t floor = data[1];
                protocol_callbacks.photo_sensor_callback(floor);
            }
            break;
            
        case CMD_DOOR_OPEN:
        case CMD_DOOR_CLOSE:
            if (length >= 3 && protocol_callbacks.door_command_callback) {
                uint8_t action = (cmd == CMD_DOOR_OPEN) ? 1 : 0;
                uint8_t floor = data[1];
                protocol_callbacks.door_command_callback(action, floor);
            }
            break;
            
        case CMD_FLOOR_CALL:
            if (length >= 3 && protocol_callbacks.floor_call_callback) {
                uint8_t floor = data[1];
                uint8_t direction = data[2];
                protocol_callbacks.floor_call_callback(floor, direction);
            }
            break;
            
        case CMD_STATUS_REQUEST:
            if (protocol_callbacks.status_request_callback) {
                protocol_callbacks.status_request_callback();
            }
            break;
            
        default:
            printf("RS485 Protocol: Unknown command 0x%02X\n", cmd);
            break;
    }
}

/* 发送光电传感器触发事件 */
bool RS485_Protocol_SendPhotoSensor(uint8_t floor)
{
    uint8_t packet[4];
    packet[0] = CMD_PHOTO_SENSOR;
    packet[1] = floor;
    packet[2] = 0;  // Reserved
    packet[3] = 0;  // Reserved
    
    rs485_status_t status = rs485_send_packet_dma(packet, 4);
    return (status == RS485_OK);
}

/* 发送门控命令 */
bool RS485_Protocol_SendDoorCommand(uint8_t action, uint8_t floor)
{
    uint8_t packet[3];
    packet[0] = (action == 1) ? CMD_DOOR_OPEN : CMD_DOOR_CLOSE;
    packet[1] = floor;
    packet[2] = 0;  // Reserved
    
    rs485_status_t status = rs485_send_packet_dma(packet, 3);
    return (status == RS485_OK);
}

/* 发送楼层呼叫 */
bool RS485_Protocol_SendFloorCall(uint8_t floor, uint8_t direction)
{
    uint8_t packet[3];
    packet[0] = CMD_FLOOR_CALL;
    packet[1] = floor;
    packet[2] = direction;  // 0=none, 1=up, 2=down
    
    rs485_status_t status = rs485_send_packet_dma(packet, 3);
    return (status == RS485_OK);
}

/* 发送状态信息 */
bool RS485_Protocol_SendStatus(uint8_t current_floor, uint8_t door_status, uint8_t moving)
{
    uint8_t packet[5];
    packet[0] = CMD_STATUS_RESPONSE;
    packet[1] = current_floor;
    packet[2] = door_status;  // 0=closed, 1=open, 2=opening, 3=closing
    packet[3] = moving;       // 0=stopped, 1=moving_up, 2=moving_down
    packet[4] = 0;            // Reserved
    
    rs485_status_t status = rs485_send_packet_dma(packet, 5);
    return (status == RS485_OK);
}

/* 发送错误报告 */
bool RS485_Protocol_SendError(uint8_t error_code)
{
    uint8_t packet[2];
    packet[0] = CMD_ERROR;
    packet[1] = error_code;
    
    rs485_status_t status = rs485_send_packet_dma(packet, 2);
    return (status == RS485_OK);
}