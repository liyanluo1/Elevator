#ifndef __CALIBRATION_H
#define __CALIBRATION_H

#include "motor_advanced.h"
#include "blackboard.h"
#include <stdbool.h>

// 校准状态
typedef enum {
    CALIB_STATE_IDLE = 0,
    CALIB_STATE_HOMING,         // 寻找原点
    CALIB_STATE_WAIT_SENSOR,    // 等待传感器触发
    CALIB_STATE_BACKOFF,        // 反向离开传感器
    CALIB_STATE_APPROACH,       // 慢速接近传感器
    CALIB_STATE_SET_ZERO,       // 设置零点
    CALIB_STATE_MOVE_TO_FLOOR,  // 移动到指定楼层
    CALIB_STATE_VERIFY,         // 验证位置
    CALIB_STATE_COMPLETE,       // 校准完成
    CALIB_STATE_ERROR           // 校准错误
} CalibState_t;

// 校准控制结构体
typedef struct {
    CalibState_t state;
    CalibState_t prev_state;
    
    MotorAdvanced_t* motor_adv;
    
    // 校准参数
    uint16_t homing_speed;      // 归位速度
    uint16_t approach_speed;    // 接近速度
    int32_t backoff_distance;   // 反向距离
    uint32_t timeout;           // 超时时间
    
    // 状态变量
    uint32_t start_time;        // 开始时间
    uint32_t state_time;        // 状态计时
    uint8_t retry_count;        // 重试次数
    bool sensor_triggered;      // 传感器触发标志
    uint8_t target_floor;       // 目标楼层
    
    // 校准结果
    bool is_calibrated;         // 是否校准成功
    int32_t floor_positions[MAX_FLOORS];  // 各楼层位置
    
} Calibration_t;

// 初始化
void Calibration_Init(Calibration_t* calib, MotorAdvanced_t* motor_adv);

// 开始校准
void Calibration_Start(Calibration_t* calib);

// 主处理函数
void Calibration_Process(Calibration_t* calib);

// 传感器事件处理
void Calibration_OnSensorTriggered(Calibration_t* calib, uint8_t floor);

// 状态查询
bool Calibration_IsComplete(Calibration_t* calib);
bool Calibration_IsRunning(Calibration_t* calib);
bool Calibration_HasError(Calibration_t* calib);
CalibState_t Calibration_GetState(Calibration_t* calib);

// 楼层位置管理
void Calibration_SetFloorPosition(Calibration_t* calib, uint8_t floor, int32_t position);
int32_t Calibration_GetFloorPosition(Calibration_t* calib, uint8_t floor);

// 运行时校准
void Calibration_UpdateFloorPosition(Calibration_t* calib, uint8_t floor);

#endif /* __CALIBRATION_H */