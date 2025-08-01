#include "ring_buffer.h"

/* 初始化环形缓冲区 */
void RingBuffer_Init(RingBuffer_t* rb, uint8_t* buffer, uint16_t size) {
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

/* 写入单个字节 */
bool RingBuffer_Put(RingBuffer_t* rb, uint8_t data) {
    if (RingBuffer_IsFull(rb)) {
        return false;
    }
    
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % rb->size;
    rb->count++;
    
    return true;
}

/* 读取单个字节 */
bool RingBuffer_Get(RingBuffer_t* rb, uint8_t* data) {
    if (RingBuffer_IsEmpty(rb)) {
        return false;
    }
    
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    rb->count--;
    
    return true;
}

/* 写入多个字节 */
bool RingBuffer_Write(RingBuffer_t* rb, const uint8_t* data, uint16_t length) {
    if (RingBuffer_GetSpace(rb) < length) {
        return false;
    }
    
    for (uint16_t i = 0; i < length; i++) {
        RingBuffer_Put(rb, data[i]);
    }
    
    return true;
}

/* 读取多个字节 */
uint16_t RingBuffer_Read(RingBuffer_t* rb, uint8_t* data, uint16_t max_length) {
    uint16_t read_count = 0;
    
    while (read_count < max_length && !RingBuffer_IsEmpty(rb)) {
        RingBuffer_Get(rb, &data[read_count]);
        read_count++;
    }
    
    return read_count;
}

/* 检查是否为空 */
bool RingBuffer_IsEmpty(RingBuffer_t* rb) {
    return (rb->count == 0);
}

/* 检查是否已满 */
bool RingBuffer_IsFull(RingBuffer_t* rb) {
    return (rb->count == rb->size);
}

/* 获取当前数据量 */
uint16_t RingBuffer_GetCount(RingBuffer_t* rb) {
    return rb->count;
}

/* 获取剩余空间 */
uint16_t RingBuffer_GetSpace(RingBuffer_t* rb) {
    return rb->size - rb->count;
}

/* 清空缓冲区 */
void RingBuffer_Clear(RingBuffer_t* rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

/* 查看数据但不移除 */
bool RingBuffer_Peek(RingBuffer_t* rb, uint8_t* data, uint16_t offset) {
    if (offset >= rb->count) {
        return false;
    }
    
    uint16_t index = (rb->tail + offset) % rb->size;
    *data = rb->buffer[index];
    
    return true;
}