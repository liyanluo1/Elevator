#ifndef __PHOTO_SENSOR_H
#define __PHOTO_SENSOR_H

#include "main.h"

/* Photo sensor pin definition - PB5 */
#define PHOTO_SENSOR_GPIO_PORT  GPIOB
#define PHOTO_SENSOR_GPIO_PIN   GPIO_PIN_5
#define PHOTO_SENSOR_EXTI_LINE  EXTI_Line5
#define PHOTO_SENSOR_IRQn       EXTI9_5_IRQn  // PB5 uses EXTI9_5

/* Photo sensor states */
typedef enum {
    PHOTO_SENSOR_BLOCKED = 0,  /* Object detected (Dark-ON) */
    PHOTO_SENSOR_CLEAR = 1     /* No object (Light-ON) */
} photo_sensor_state_t;

/* Function prototypes */
void PhotoSensor_Init(void);
photo_sensor_state_t PhotoSensor_GetState(void);
uint32_t PhotoSensor_GetTriggerCount(void);
void PhotoSensor_ResetCount(void);
void PhotoSensor_IRQHandler(void);

/* Callback - can be implemented by user */
void PhotoSensor_TriggerCallback(void);

/* Floor detection functions */
typedef struct {
    uint8_t current_floor;
    uint8_t moving_direction;  /* 0=stopped, 1=up, 2=down */
    uint32_t last_trigger_time;
    void (*floor_detected_callback)(uint8_t floor);
} floor_detector_t;

void PhotoSensor_InitFloorDetector(floor_detector_t *detector, uint8_t initial_floor);
void PhotoSensor_SetDirection(floor_detector_t *detector, uint8_t direction);
void PhotoSensor_ProcessTrigger(floor_detector_t *detector);

#endif /* __PHOTO_SENSOR_H */