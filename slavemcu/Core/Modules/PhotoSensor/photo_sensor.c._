#include "photo_sensor.h"
#include "rs485_slave_adapter.h"
#include <stdio.h>
#include <stdlib.h>

// 全局光电传感器实例
PhotoSensor_t g_photo_sensor;

// 楼层位置信息
FloorPosition_t g_floor_positions[MAX_FLOORS] = {
    {1, FLOOR_1_POSITION, FLOOR_1_POSITION, false},
    {2, FLOOR_2_POSITION, FLOOR_2_POSITION, false},
    {3, FLOOR_3_POSITION, FLOOR_3_POSITION, false}
};

// 初始化光电传感器模块
void PhotoSensor_Init(void) {
    // 清零结构体
    g_photo_sensor.current_state = PHOTO_STATE_IDLE;
    g_photo_sensor.prev_state = PHOTO_STATE_IDLE;
    g_photo_sensor.trigger_time = 0;
    g_photo_sensor.state_entry_time = HAL_GetTick();
    g_photo_sensor.trigger_count = 0;
    g_photo_sensor.debounce_count = 0;
    g_photo_sensor.last_trigger = false;
    g_photo_sensor.detected_floor = 1;
    g_photo_sensor.motion_direction = DIR_IDLE;
    g_photo_sensor.pulse_position = FLOOR_1_POSITION;
    g_photo_sensor.last_pulse_position = FLOOR_1_POSITION;
    g_photo_sensor.is_calibrating = false;
    g_photo_sensor.calibration_start = 0;
    g_photo_sensor.calibration_offset = 0;
    g_photo_sensor.sensor_isr_flag = false;
    g_photo_sensor.error_count = 0;
    g_photo_sensor.max_errors = 3;
    
    // 初始化GPIO和中断
    PhotoSensor_GPIO_Init();
    
    // 设置第一楼为已校准（基准楼层）
    g_floor_positions[0].is_calibrated = true;
}

// 主更新函数
void PhotoSensor_Update(void) {
    // 检查中断标志
    if (g_photo_sensor.sensor_isr_flag) {
        g_photo_sensor.sensor_isr_flag = false;
        g_photo_sensor.trigger_time = HAL_GetTick();
        
        // 更新黑板
        LocalBlackboard_UpdateSensorState(true);
    }
    
    // 状态机处理
    switch (g_photo_sensor.current_state) {
        case PHOTO_STATE_IDLE:
            PhotoSensor_HandleIdle();
            break;
            
        case PHOTO_STATE_DETECTING:
            PhotoSensor_HandleDetecting();
            break;
            
        case PHOTO_STATE_DEBOUNCE:
            PhotoSensor_HandleDebounce();
            break;
            
        case PHOTO_STATE_CALIBRATE:
            PhotoSensor_HandleCalibrate();
            break;
            
        case PHOTO_STATE_TIMEOUT:
            PhotoSensor_HandleTimeout();
            break;
            
        case PHOTO_STATE_ERROR:
            PhotoSensor_HandleError();
            break;
            
        default:
            g_photo_sensor.current_state = PHOTO_STATE_IDLE;
            break;
    }
    
    // 更新状态进入时间
    if (g_photo_sensor.current_state != g_photo_sensor.prev_state) {
        g_photo_sensor.state_entry_time = HAL_GetTick();
        g_photo_sensor.prev_state = g_photo_sensor.current_state;
    }
}

// 开始检测
void PhotoSensor_StartDetecting(void) {
    g_photo_sensor.current_state = PHOTO_STATE_DETECTING;
    g_photo_sensor.trigger_count = 0;
    g_photo_sensor.error_count = 0;
}

// 停止检测
void PhotoSensor_StopDetecting(void) {
    g_photo_sensor.current_state = PHOTO_STATE_IDLE;
}

// 开始校准
void PhotoSensor_StartCalibration(void) {
    g_photo_sensor.current_state = PHOTO_STATE_CALIBRATE;
    g_photo_sensor.is_calibrating = true;
    g_photo_sensor.calibration_start = HAL_GetTick();
}

// 检查是否已校准
bool PhotoSensor_IsCalibrated(void) {
    // 检查所有楼层是否都已校准
    for (int i = 0; i < MAX_FLOORS; i++) {
        if (!g_floor_positions[i].is_calibrated) {
            return false;
        }
    }
    return true;
}

// 处理检测逻辑
void PhotoSensor_ProcessDetection(void) {
    bool sensor_state = PhotoSensor_ReadSensor();
    
    // 边沿检测
    if (sensor_state && !g_photo_sensor.last_trigger) {
        // 上升沿 - 进入去抖状态
        g_photo_sensor.current_state = PHOTO_STATE_DEBOUNCE;
        g_photo_sensor.debounce_count = 0;
    }
    
    g_photo_sensor.last_trigger = sensor_state;
}

// 根据方向和触发次数计算楼层
uint8_t PhotoSensor_CalculateFloor(Direction_t dir, int trigger_count) {
    uint8_t floor = g_photo_sensor.detected_floor;
    
    // 从黑板获取运动方向
    if (g_local_bb.last_direction != DIR_IDLE) {
        dir = g_local_bb.last_direction;
    }
    
    // 基于脉冲位置计算楼层
    if (abs(g_photo_sensor.pulse_position - FLOOR_1_POSITION) < FLOOR_PULSE_TOLERANCE) {
        floor = 1;
    } else if (abs(g_photo_sensor.pulse_position - FLOOR_2_POSITION) < FLOOR_PULSE_TOLERANCE) {
        floor = 2;
    } else if (abs(g_photo_sensor.pulse_position - FLOOR_3_POSITION) < FLOOR_PULSE_TOLERANCE) {
        floor = 3;
    } else {
        // 根据方向和触发次数估算
        if (dir == DIR_UP) {
            floor = g_photo_sensor.detected_floor + 1;
            if (floor > MAX_FLOORS) floor = MAX_FLOORS;
        } else if (dir == DIR_DOWN) {
            floor = g_photo_sensor.detected_floor - 1;
            if (floor < 1) floor = 1;
        }
    }
    
    return floor;
}

// 更新位置
void PhotoSensor_UpdatePosition(int32_t pulse_count) {
    g_photo_sensor.last_pulse_position = g_photo_sensor.pulse_position;
    g_photo_sensor.pulse_position = pulse_count;
}

// 校准楼层
void PhotoSensor_CalibrateFloor(uint8_t floor, int32_t actual_position) {
    if (floor >= 1 && floor <= MAX_FLOORS) {
        g_floor_positions[floor - 1].actual_position = actual_position;
        g_floor_positions[floor - 1].is_calibrated = true;
        
        // 计算偏移
        g_photo_sensor.calibration_offset = actual_position - g_floor_positions[floor - 1].theoretical_position;
        
        // 更新黑板
        g_local_bb.position_offset = g_photo_sensor.calibration_offset;
        LocalBlackboard_MarkForSync(SYNC_FIELD_POSITION_OFFSET);
    }
}

// 计算位置偏移
int32_t PhotoSensor_CalculateOffset(uint8_t floor, int32_t current_position) {
    if (floor >= 1 && floor <= MAX_FLOORS) {
        return current_position - g_floor_positions[floor - 1].theoretical_position;
    }
    return 0;
}

// GPIO初始化
void PhotoSensor_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIO时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    #if USE_NO_SIGNAL
    // 配置常开信号（黑线）- NPN输出，遮挡时为低电平
    // 上拉输入，下降沿触发（物体遮挡时触发）
    GPIO_InitStruct.Pin = PHOTO_SENSOR_NO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PHOTO_SENSOR_NO_GPIO_Port, &GPIO_InitStruct);
    
    // 配置中断优先级
    HAL_NVIC_SetPriority(PHOTO_SENSOR_NO_IRQ, 2, 0);
    HAL_NVIC_EnableIRQ(PHOTO_SENSOR_NO_IRQ);
    #endif
    
    #if USE_NC_SIGNAL
    // 配置常闭信号（白线）- NPN输出，未遮挡时为低电平
    // 上拉输入，上升沿触发（物体离开时触发）
    GPIO_InitStruct.Pin = PHOTO_SENSOR_NC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PHOTO_SENSOR_NC_GPIO_Port, &GPIO_InitStruct);
    
    // 配置中断优先级
    HAL_NVIC_SetPriority(PHOTO_SENSOR_NC_IRQ, 2, 0);
    HAL_NVIC_EnableIRQ(PHOTO_SENSOR_NC_IRQ);
    #endif
}

// 中断服务程序
void PhotoSensor_ISR(void) {
    #if USE_NO_SIGNAL
    if (__HAL_GPIO_EXTI_GET_IT(PHOTO_SENSOR_NO_Pin) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(PHOTO_SENSOR_NO_Pin);
        g_photo_sensor.sensor_isr_flag = true;
    }
    #endif
    
    #if USE_NC_SIGNAL
    if (__HAL_GPIO_EXTI_GET_IT(PHOTO_SENSOR_NC_Pin) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(PHOTO_SENSOR_NC_Pin);
        g_photo_sensor.sensor_isr_flag = true;
    }
    #endif
}

// 读取常开信号（黑线）
bool PhotoSensor_ReadNO(void) {
    // NPN常开：未遮挡=高电平（上拉），遮挡=低电平
    return (HAL_GPIO_ReadPin(PHOTO_SENSOR_NO_GPIO_Port, PHOTO_SENSOR_NO_Pin) == GPIO_PIN_RESET);
}

// 读取常闭信号（白线）
bool PhotoSensor_ReadNC(void) {
    // NPN常闭：未遮挡=低电平，遮挡=高电平（上拉）
    return (HAL_GPIO_ReadPin(PHOTO_SENSOR_NC_GPIO_Port, PHOTO_SENSOR_NC_Pin) == GPIO_PIN_SET);
}

// 读取传感器状态（根据配置选择信号）
bool PhotoSensor_ReadSensor(void) {
    #if USE_NO_SIGNAL
    return PhotoSensor_ReadNO();
    #else
    return PhotoSensor_ReadNC();
    #endif
}

// 获取当前状态
PhotoState_t PhotoSensor_GetState(void) {
    return g_photo_sensor.current_state;
}

// 获取当前楼层
uint8_t PhotoSensor_GetCurrentFloor(void) {
    return g_photo_sensor.detected_floor;
}

// 获取位置
int32_t PhotoSensor_GetPosition(void) {
    return g_photo_sensor.pulse_position;
}

// 检查是否有错误
bool PhotoSensor_HasError(void) {
    return (g_photo_sensor.current_state == PHOTO_STATE_ERROR) || 
           (g_photo_sensor.error_count >= g_photo_sensor.max_errors);
}

// 打印状态
void PhotoSensor_PrintStatus(void) {
    printf("=== Photo Sensor Status ===\n");
    printf("State: %d\n", g_photo_sensor.current_state);
    printf("Detected Floor: %d\n", g_photo_sensor.detected_floor);
    printf("Position: %ld\n", g_photo_sensor.pulse_position);
    printf("Trigger Count: %d\n", g_photo_sensor.trigger_count);
    printf("Error Count: %d\n", g_photo_sensor.error_count);
    printf("Is Calibrating: %s\n", g_photo_sensor.is_calibrating ? "Yes" : "No");
    printf("==========================\n");
}

// 状态处理函数 - 空闲
void PhotoSensor_HandleIdle(void) {
    // 等待启动命令或检测到传感器触发
    if (g_photo_sensor.sensor_isr_flag) {
        g_photo_sensor.current_state = PHOTO_STATE_DETECTING;
    }
}

// 状态处理函数 - 检测中
void PhotoSensor_HandleDetecting(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 检查超时
    if (current_time - g_photo_sensor.state_entry_time > SENSOR_TIMEOUT) {
        g_photo_sensor.current_state = PHOTO_STATE_TIMEOUT;
        return;
    }
    
    // 处理检测逻辑
    PhotoSensor_ProcessDetection();
}

// 状态处理函数 - 去抖动
void PhotoSensor_HandleDebounce(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 去抖时间检查
    if (current_time - g_photo_sensor.trigger_time >= SENSOR_DEBOUNCE_TIME) {
        // 再次确认传感器状态
        if (PhotoSensor_ReadSensor()) {
            // 确认触发
            g_photo_sensor.trigger_count++;
            
            // 获取运动方向
            Direction_t dir = g_local_bb.last_direction;
            
            // 计算楼层
            uint8_t floor = PhotoSensor_CalculateFloor(dir, g_photo_sensor.trigger_count);
            
            if (floor != g_photo_sensor.detected_floor) {
                g_photo_sensor.detected_floor = floor;
                
                // 更新黑板
                LocalBlackboard_SetCurrentFloor(floor);
                
                // 发送RS485消息
                RS485_Slave_SendSensorData(floor, PhotoSensor_CalculateOffset(floor, g_photo_sensor.pulse_position));
                
                // 如果需要校准
                if (g_photo_sensor.is_calibrating) {
                    PhotoSensor_CalibrateFloor(floor, g_photo_sensor.pulse_position);
                }
            }
            
            // 推送位置调整事件（如果有偏移）
            int32_t offset = PhotoSensor_CalculateOffset(floor, g_photo_sensor.pulse_position);
            if (abs(offset) > FLOOR_PULSE_TOLERANCE) {
                LocalBlackboard_PushEvent(EVENT_POSITION_ADJUST, offset);
            }
        }
        
        // 返回检测状态
        g_photo_sensor.current_state = PHOTO_STATE_DETECTING;
    }
}

// 状态处理函数 - 校准
void PhotoSensor_HandleCalibrate(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 校准超时检查
    if (current_time - g_photo_sensor.calibration_start > SENSOR_CALIBRATE_TIME) {
        g_photo_sensor.is_calibrating = false;
        
        // 检查校准结果
        if (PhotoSensor_IsCalibrated()) {
            g_photo_sensor.current_state = PHOTO_STATE_IDLE;
            LocalBlackboard_SetState(ELEVATOR_IDLE);
        } else {
            g_photo_sensor.current_state = PHOTO_STATE_ERROR;
            g_photo_sensor.error_count++;
        }
    }
    
    // 在校准期间继续检测
    PhotoSensor_ProcessDetection();
}

// 状态处理函数 - 超时
void PhotoSensor_HandleTimeout(void) {
    g_photo_sensor.error_count++;
    
    // 推送错误事件
    LocalBlackboard_PushEvent(EVENT_ERROR, 0x200); // 传感器超时错误
    
    // 重试或进入错误状态
    if (g_photo_sensor.error_count < g_photo_sensor.max_errors) {
        g_photo_sensor.current_state = PHOTO_STATE_IDLE;
    } else {
        g_photo_sensor.current_state = PHOTO_STATE_ERROR;
    }
}

// 状态处理函数 - 错误
void PhotoSensor_HandleError(void) {
    static uint32_t error_report_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 定期报告错误
    if (current_time - error_report_time > 5000) {
        error_report_time = current_time;
        
        // 更新黑板错误码
        g_local_bb.error_code = 0x201; // 光电传感器故障
        LocalBlackboard_MarkForSync(SYNC_FIELD_ERROR);
        
        // 发送错误报告
        RS485_SlaveAdapter_SendError(0x201);
    }
    
    // 检查恢复条件
    if (PhotoSensor_ReadSensor()) {
        // 尝试恢复
        g_photo_sensor.error_count = 0;
        g_photo_sensor.current_state = PHOTO_STATE_IDLE;
    }
}