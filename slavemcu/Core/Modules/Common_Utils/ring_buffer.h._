#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* 环形缓冲区结构体 */
typedef struct {
    uint8_t* buffer;        // 数据缓冲区
    uint16_t size;          // 缓冲区大小
    uint16_t head;          // 写入位置
    uint16_t tail;          // 读取位置
    uint16_t count;         // 当前数据量
} RingBuffer_t;

/* 函数声明 */
void RingBuffer_Init(RingBuffer_t* rb, uint8_t* buffer, uint16_t size);
bool RingBuffer_Put(RingBuffer_t* rb, uint8_t data);
bool RingBuffer_Get(RingBuffer_t* rb, uint8_t* data);
bool RingBuffer_Write(RingBuffer_t* rb, const uint8_t* data, uint16_t length);
uint16_t RingBuffer_Read(RingBuffer_t* rb, uint8_t* data, uint16_t max_length);
bool RingBuffer_IsEmpty(RingBuffer_t* rb);
bool RingBuffer_IsFull(RingBuffer_t* rb);
uint16_t RingBuffer_GetCount(RingBuffer_t* rb);
uint16_t RingBuffer_GetSpace(RingBuffer_t* rb);
void RingBuffer_Clear(RingBuffer_t* rb);
bool RingBuffer_Peek(RingBuffer_t* rb, uint8_t* data, uint16_t offset);

/* 静态分配宏 */
#define RING_BUFFER_DECLARE(name, size) \
    static uint8_t name##_buffer[size]; \
    static RingBuffer_t name = { \
        .buffer = name##_buffer, \
        .size = size, \
        .head = 0, \
        .tail = 0, \
        .count = 0 \
    }

#endif /* RING_BUFFER_H */