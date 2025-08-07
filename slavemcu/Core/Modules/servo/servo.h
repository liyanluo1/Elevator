#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// 舵机寄存器地址（从提供的内存表）
#define SERVO_REG_ID                0x05  // ID
#define SERVO_REG_BAUDRATE          0x06  // 波特率 (0=1M, 1=500K 等)
#define SERVO_REG_RETURN_DELAY      0x07  // 返回延时
#define SERVO_REG_MIN_ANGLE         0x09  // 最小角度限制 (2字节)
#define SERVO_REG_MAX_ANGLE         0x0B  // 最大角度限制 (2字节)
#define SERVO_REG_MAX_TEMP          0x0D  // 最高温度
#define SERVO_REG_MAX_VOLT          0x0E  // 最高电压
#define SERVO_REG_MIN_VOLT          0x0F  // 最低电压
#define SERVO_REG_MAX_TORQUE        0x10  // 最大扭矩 (2字节)
#define SERVO_REG_PHASE             0x12  // 相位
#define SERVO_REG_UNLOAD_COND       0x13  // 卸载条件
#define SERVO_REG_LED_ALARM         0x14  // LED报警条件
#define SERVO_REG_POS_P             0x15  // 位置P
#define SERVO_REG_POS_D             0x16  // 位置D
#define SERVO_REG_POS_I             0x17  // 位置I
#define SERVO_REG_MIN_START_TORQUE  0x18  // 最小启动力
#define SERVO_REG_INTEGRAL_LIMIT    0x19  // 积分限制
#define SERVO_REG_CW_DEADZONE       0x1A  // 顺时针死区
#define SERVO_REG_CCW_DEADZONE      0x1B  // 逆时针死区
#define SERVO_REG_PROTECT_CURR      0x1C  // 保护电流 (2字节)
#define SERVO_REG_ANGLE_RES         0x1E  // 角度分辨率
#define SERVO_REG_POS_CORRECT       0x1F  // 位置校正 (2字节)
#define SERVO_REG_MODE              0x21  // 运行模式 (0=位置)
#define SERVO_REG_PROTECT_TORQUE    0x22  // 保护扭矩
#define SERVO_REG_PROTECT_TIME      0x23  // 保护时间
#define SERVO_REG_OVERLOAD_TORQUE   0x24  // 过载扭矩
#define SERVO_REG_SPEED_P           0x25  // 速度P
#define SERVO_REG_OVERCURR_TIME     0x26  // 过流保护时间
#define SERVO_REG_SPEED_I           0x27  // 速度I
#define SERVO_REG_TORQUE_ENABLE     0x28  // 扭矩开关 (0=关,1=开)
#define SERVO_REG_ACCEL             0x29  // 加速度
#define SERVO_REG_TARGET_POS        0x2A  // 目标位置 (2字节)
#define SERVO_REG_RUNTIME           0x2C  // 运行时间 (2字节，PWM模式)
#define SERVO_REG_SPEED             0x2E  // 运行速度 (2字节)
#define SERVO_REG_TORQUE_LIMIT      0x30  // 转矩限制 (2字节)
#define SERVO_REG_LOCK_FLAG         0x37  // 锁标志
#define SERVO_REG_CURR_POS          0x38  // 当前位置 (2字节，只读)
#define SERVO_REG_CURR_SPEED        0x3A  // 当前速度 (2字节，只读)
#define SERVO_REG_CURR_LOAD         0x3C  // 当前负载 (2字节，只读)
#define SERVO_REG_CURR_VOLT         0x3E  // 当前电压 (只读)
#define SERVO_REG_CURR_TEMP         0x3F  // 当前温度 (只读)
#define SERVO_REG_ASYNC_FLAG        0x40  // 异步写标志 (只读)
#define SERVO_REG_STATUS            0x41  // 舵机状态 (只读)
#define SERVO_REG_MOVING            0x42  // 移动标志 (只读)
#define SERVO_REG_CURR_CURR         0x45  // 当前电流 (2字节，只读)

// 指令码
#define SERVO_INST_PING             0x01
#define SERVO_INST_READ             0x02
#define SERVO_INST_WRITE            0x03
#define SERVO_INST_REG_WRITE        0x04
#define SERVO_INST_ACTION           0x05
#define SERVO_INST_SYNC_WRITE       0x83

// 默认ID和广播ID
#define SERVO_DEFAULT_ID            1
#define SERVO_BROADCAST_ID          0xFE

// 函数原型
void servo_init(UART_HandleTypeDef *huart);  // 初始化（设置波特率等）
uint8_t servo_ping(uint8_t id);  // PING检测
void servo_write_reg(uint8_t id, uint8_t reg_addr, uint8_t *data, uint8_t data_len);  // 写寄存器
uint8_t servo_read_reg(uint8_t id, uint8_t reg_addr, uint8_t data_len, uint8_t *rx_data);  // 读寄存器
void servo_set_torque_enable(uint8_t id, uint8_t enable);  // 扭矩开关
void servo_set_speed(uint8_t id, int16_t speed);  // 设置速度 (-32766~32766)
void servo_set_position(uint8_t id, int16_t position);  // 设置目标位置 (-32766~32766)
uint16_t servo_get_position(uint8_t id);  // 获取当前位置
uint16_t servo_get_position_debug(uint8_t id);  // 获取当前位置（调试版）
uint8_t servo_is_moving(uint8_t id);  // 检查是否在移动
void servo_demo_max_speed_360(uint8_t id);  // Demo: 最大速度转360度

#endif /* SERVO_H */
