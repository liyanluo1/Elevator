#ifndef __PHOTO_SENSOR_H
#define __PHOTO_SENSOR_H

#include "main.h"

/* Photo sensor pin definition - Changed to PB3 */
#define PHOTO_SENSOR_GPIO_PORT  GPIOB
#define PHOTO_SENSOR_GPIO_PIN   GPIO_PIN_3
#define PHOTO_SENSOR_EXTI_LINE  EXTI_Line3
#define PHOTO_SENSOR_IRQn       EXTI3_IRQn

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

#endif /* __PHOTO_SENSOR_H */