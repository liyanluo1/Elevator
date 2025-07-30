#include "servo.h"
#include <string.h>  // 用于memcpy

static UART_HandleTypeDef *servo_uart;  // USART句柄

// 函数：计算校验和
static uint8_t calculate_checksum(uint8_t *packet, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 2; i < len - 1; i++) {  // 从ID到Params结束
        sum += packet[i];
    }
    return ~sum;
}

// 函数：发送包
static void send_packet(uint8_t *packet, uint8_t len) {
    HAL_UART_Transmit(servo_uart, packet, len, HAL_MAX_DELAY);
}

// 函数：接收包（简单阻塞式，假设响应快速；实际可加超时/DMA）
static uint8_t receive_packet(uint8_t *rx_packet, uint8_t expected_len) {
    return HAL_UART_Receive(servo_uart, rx_packet, expected_len, 100);  // 返回实际接收长度，超时100ms
}

// 初始化（默认波特率1M，无需设置寄存器0x06，因为初始值0=1M）
void servo_init(UART_HandleTypeDef *huart) {
    servo_uart = huart;
    // 可在此PING舵机确认连接
    servo_ping(SERVO_DEFAULT_ID);
}

// PING
uint8_t servo_ping(uint8_t id) {
    uint8_t packet[6] = {0xFF, 0xFF, id, 0x02, SERVO_INST_PING, 0x00};
    packet[5] = calculate_checksum(packet, 6);
    send_packet(packet, 6);

    uint8_t rx_packet[6];
    if (receive_packet(rx_packet, 6) == 6 &&
        rx_packet[0] == 0xFF && rx_packet[1] == 0xFF && rx_packet[2] == id &&
        rx_packet[3] == 0x02 && rx_packet[4] == 0x00) {
        return 1;  // 成功
    }
    return 0;
}

// 写寄存器（低字节在前）
void servo_write_reg(uint8_t id, uint8_t reg_addr, uint8_t *data, uint8_t data_len) {
    uint8_t packet[256];
    uint8_t idx = 0;
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFF;
    packet[idx++] = id;
    packet[idx++] = data_len + 3;  // Length = Instr + Addr + Data
    packet[idx++] = SERVO_INST_WRITE;
    packet[idx++] = reg_addr;
    memcpy(&packet[idx], data, data_len);
    idx += data_len;
    packet[idx] = calculate_checksum(packet, idx + 1);
    send_packet(packet, idx + 1);

    // 忽略响应（若非广播），实际可检查
    if (id != SERVO_BROADCAST_ID) {
        uint8_t rx_packet[6];
        receive_packet(rx_packet, 6);  // 预期响应Length=2, Error+Checksum
    }
}

// 读寄存器
uint8_t servo_read_reg(uint8_t id, uint8_t reg_addr, uint8_t data_len, uint8_t *rx_data) {
    uint8_t packet[7] = {0xFF, 0xFF, id, 0x04, SERVO_INST_READ, reg_addr, data_len};
    packet[6] = calculate_checksum(packet, 7);
    send_packet(packet, 7);

    uint8_t expected_len = data_len + 5;  // FF FF ID Len Error Params... Checksum
    uint8_t rx_packet[256];
    if (receive_packet(rx_packet, expected_len) == expected_len &&
        rx_packet[0] == 0xFF && rx_packet[1] == 0xFF && rx_packet[2] == id &&
        rx_packet[3] == data_len + 2 && rx_packet[4] == 0x00) {
        memcpy(rx_data, &rx_packet[5], data_len);
        return 1;  // 成功
    }
    return 0;
}

// 扭矩开关
void servo_set_torque_enable(uint8_t id, uint8_t enable) {
    uint8_t data = enable;
    servo_write_reg(id, SERVO_REG_TORQUE_ENABLE, &data, 1);
}

// 设置速度 (2字节，低在前)
void servo_set_speed(uint8_t id, int16_t speed) {
    uint8_t data[2];
    data[0] = speed & 0xFF;  // 低
    data[1] = (speed >> 8) & 0xFF;  // 高
    servo_write_reg(id, SERVO_REG_SPEED, data, 2);
}

// 设置位置 (2字节，低在前)
void servo_set_position(uint8_t id, int16_t position) {
    uint8_t data[2];
    data[0] = position & 0xFF;
    data[1] = (position >> 8) & 0xFF;
    servo_write_reg(id, SERVO_REG_TARGET_POS, data, 2);
}

// 获取当前位置
uint16_t servo_get_position(uint8_t id) {
    uint8_t rx_data[2];
    if (servo_read_reg(id, SERVO_REG_CURR_POS, 2, rx_data)) {
        return (rx_data[1] << 8) | rx_data[0];  // 高<<8 | 低
    }
    return 0;  // 错误返回0
}

// 检查是否移动
uint8_t servo_is_moving(uint8_t id) {
    uint8_t rx_data;
    if (servo_read_reg(id, SERVO_REG_MOVING, 1, &rx_data)) {
        return rx_data;
    }
    return 0;
}

// Demo: 最大速度转360度 (正向最大速度转4096步，假设位置模式)
void servo_demo_max_speed_360(uint8_t id) {
    servo_set_torque_enable(id, 1);  // 开启扭矩

    // 设置最大速度 (32767 正向)
    servo_set_speed(id, 32767);

    // 获取当前位置，然后加4096步 (360度=4096步)
    int16_t curr_pos = (int16_t)servo_get_position(id);
    int16_t target_pos = curr_pos + 4096;

    servo_set_position(id, target_pos);

    // 等待移动完成 (轮询)
    while (servo_is_moving(id)) {
        HAL_Delay(10);
    }
}
