#ifndef __PHOTO_SENSOR_H
#define __PHOTO_SENSOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "local_blackboard.h"

// GPIO配置（光电传感器）
#define PHOTO_SENSOR_GPIO_Port   GPIOC
#define PHOTO_SENSOR_Pin         GPIO_PIN_0
#define PHOTO_SENSOR_IRQ         EXTI0_IRQn

// 传感器参数
#define SENSOR_DEBOUNCE_TIME     50      // 去抖时间（ms）
#define SENSOR_TIMEOUT           5000    // 超时时间（ms）
#define SENSOR_CALIBRATE_TIME    2000    // 校准时间（ms）
#define FLOOR_PULSE_TOLERANCE    100     // 楼层脉冲容差

// 楼层位置定义（理论脉冲数）
#define FLOOR_1_POSITION         0       // 1楼位置（基准）
#define FLOOR_2_POSITION         10000   // 2楼位置（脉冲数）
#define FLOOR_3_POSITION         20000   // 3楼位置（脉冲数）

// 光电传感器状态枚举
typedef enum {
    PHOTO_STATE_IDLE = 0,        // 空闲
    PHOTO_STATE_DETECTING,       // 检测中
    PHOTO_STATE_DEBOUNCE,        // 去抖动
    PHOTO_STATE_CALIBRATE,       // 校准中
    PHOTO_STATE_TIMEOUT,         // 超时
    PHOTO_STATE_ERROR           // 错误
} PhotoState_t;

// 光电传感器FSM结构体
typedef struct {
    // FSM状态
    PhotoState_t current_state;
    PhotoState_t prev_state;
    
    // 私有变量
    uint32_t trigger_time;          // 触发时间
    uint32_t state_entry_time;      // 状态进入时间
    int trigger_count;              // 触发计数
    int debounce_count;             // 去抖计数
    bool last_trigger;              // 上次触发状态
    
    // 楼层检测
    uint8_t detected_floor;         // 检测到的楼层
    Direction_t motion_direction;   // 运动方向
    int32_t pulse_position;         // 脉冲位置
    int32_t last_pulse_position;    // 上次脉冲位置
    
    // 校准相关
    bool is_calibrating;            // 校准标志
    uint32_t calibration_start;     // 校准开始时间
    int32_t calibration_offset;     // 校准偏移
    
    // 中断标志
    volatile bool sensor_isr_flag;  // 中断标志
    
    // 错误管理
    uint8_t error_count;            // 错误计数
    uint8_t max_errors;             // 最大错误次数
    
} PhotoSensor_t;

// 楼层信息结构
typedef struct {
    uint8_t floor_number;           // 楼层号
    int32_t theoretical_position;   // 理论位置
    int32_t actual_position;        // 实际位置
    bool is_calibrated;             // 是否已校准
} FloorPosition_t;

// 全局光电传感器实例
extern PhotoSensor_t g_photo_sensor;

// 楼层位置信息
extern FloorPosition_t g_floor_positions[MAX_FLOORS];

// 初始化和更新函数
void PhotoSensor_Init(void);
void PhotoSensor_Update(void);

// 状态控制函数
void PhotoSensor_StartDetecting(void);
void PhotoSensor_StopDetecting(void);
void PhotoSensor_StartCalibration(void);
bool PhotoSensor_IsCalibrated(void);

// 内部处理函数
void PhotoSensor_ProcessDetection(void);
uint8_t PhotoSensor_CalculateFloor(Direction_t dir, int trigger_count);
void PhotoSensor_UpdatePosition(int32_t pulse_count);
void PhotoSensor_CalibrateFloor(uint8_t floor, int32_t actual_position);
int32_t PhotoSensor_CalculateOffset(uint8_t floor, int32_t current_position);

// GPIO和中断处理
void PhotoSensor_GPIO_Init(void);
void PhotoSensor_ISR(void);
bool PhotoSensor_ReadSensor(void);

// 状态查询
PhotoState_t PhotoSensor_GetState(void);
uint8_t PhotoSensor_GetCurrentFloor(void);
int32_t PhotoSensor_GetPosition(void);
bool PhotoSensor_HasError(void);

// 调试函数
void PhotoSensor_PrintStatus(void);

// 状态机处理函数（内部）
void PhotoSensor_HandleIdle(void);
void PhotoSensor_HandleDetecting(void);
void PhotoSensor_HandleDebounce(void);
void PhotoSensor_HandleCalibrate(void);
void PhotoSensor_HandleTimeout(void);
void PhotoSensor_HandleError(void);

#endif /* __PHOTO_SENSOR_H */