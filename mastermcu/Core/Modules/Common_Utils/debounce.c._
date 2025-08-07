#include "debounce.h"

/* 初始化去抖动器 */
void Debouncer_Init(Debouncer_t* debouncer, uint32_t debounce_ms) {
    debouncer->current_state = false;
    debouncer->last_raw_state = false;
    debouncer->last_change_time = 0;
    debouncer->debounce_time = debounce_ms;
    debouncer->state_changed = false;
}

/* 更新去抖动器状态 */
bool Debouncer_Update(Debouncer_t* debouncer, bool raw_state, uint32_t current_time) {
    // 检测原始状态变化
    if (raw_state != debouncer->last_raw_state) {
        debouncer->last_raw_state = raw_state;
        debouncer->last_change_time = current_time;
    }
    
    // 检查是否超过去抖时间
    if ((current_time - debouncer->last_change_time) >= debouncer->debounce_time) {
        // 更新稳定状态
        if (debouncer->current_state != raw_state) {
            debouncer->current_state = raw_state;
            debouncer->state_changed = true;
        }
    }
    
    return debouncer->current_state;
}

/* 获取当前稳定状态 */
bool Debouncer_GetState(Debouncer_t* debouncer) {
    return debouncer->current_state;
}

/* 检查状态是否发生变化 */
bool Debouncer_HasChanged(Debouncer_t* debouncer) {
    return debouncer->state_changed;
}

/* 清除状态变化标志 */
void Debouncer_ClearChange(Debouncer_t* debouncer) {
    debouncer->state_changed = false;
}

/* 批量去抖动器初始化 */
void DebouncerArray_Init(DebouncerArray_t* array, uint8_t count, uint32_t debounce_ms) {
    array->count = (count > MAX_DEBOUNCERS) ? MAX_DEBOUNCERS : count;
    
    for (uint8_t i = 0; i < array->count; i++) {
        Debouncer_Init(&array->debouncers[i], debounce_ms);
    }
}

/* 批量更新去抖动器 */
void DebouncerArray_Update(DebouncerArray_t* array, uint32_t inputs, uint32_t current_time) {
    for (uint8_t i = 0; i < array->count; i++) {
        bool raw_state = (inputs >> i) & 0x01;
        Debouncer_Update(&array->debouncers[i], raw_state, current_time);
    }
}

/* 获取所有稳定状态 */
uint32_t DebouncerArray_GetStates(DebouncerArray_t* array) {
    uint32_t states = 0;
    
    for (uint8_t i = 0; i < array->count; i++) {
        if (array->debouncers[i].current_state) {
            states |= (1U << i);
        }
    }
    
    return states;
}

/* 获取所有变化标志 */
uint32_t DebouncerArray_GetChanges(DebouncerArray_t* array) {
    uint32_t changes = 0;
    
    for (uint8_t i = 0; i < array->count; i++) {
        if (array->debouncers[i].state_changed) {
            changes |= (1U << i);
        }
    }
    
    return changes;
}