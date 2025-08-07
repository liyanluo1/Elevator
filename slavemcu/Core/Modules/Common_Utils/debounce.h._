#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include <stdint.h>
#include <stdbool.h>

/* 去抖动器结构体 */
typedef struct {
    bool current_state;     // 当前稳定状态
    bool last_raw_state;    // 上次原始状态
    uint32_t last_change_time; // 上次变化时间
    uint32_t debounce_time;    // 去抖时间（毫秒）
    bool state_changed;        // 状态变化标志
} Debouncer_t;

/* 函数声明 */
void Debouncer_Init(Debouncer_t* debouncer, uint32_t debounce_ms);
bool Debouncer_Update(Debouncer_t* debouncer, bool raw_state, uint32_t current_time);
bool Debouncer_GetState(Debouncer_t* debouncer);
bool Debouncer_HasChanged(Debouncer_t* debouncer);
void Debouncer_ClearChange(Debouncer_t* debouncer);

/* 批量去抖动器 - 用于多个按钮/输入 */
#define MAX_DEBOUNCERS 16

typedef struct {
    Debouncer_t debouncers[MAX_DEBOUNCERS];
    uint8_t count;
} DebouncerArray_t;

void DebouncerArray_Init(DebouncerArray_t* array, uint8_t count, uint32_t debounce_ms);
void DebouncerArray_Update(DebouncerArray_t* array, uint32_t inputs, uint32_t current_time);
uint32_t DebouncerArray_GetStates(DebouncerArray_t* array);
uint32_t DebouncerArray_GetChanges(DebouncerArray_t* array);

#endif /* DEBOUNCE_H */