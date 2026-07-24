/**
 * Dual FIFO buffer module for CAN frame buffering
 *
 * FIFO_HIGH (depth=16): 0-level emergency frames only
 * FIFO_NORMAL (depth=64): 1/2-level normal + low-frequency frames
 *
 * Overflow policy:
 *   HIGH: drop oldest frame, increment overflow counter
 *   NORMAL: scan backward for level-2 frame to overwrite;
 *           if all are level-1, drop oldest frame
 *
 * Concurrency: all reads/writes protected by FreeRTOS mutex.
 */

#ifndef __FIFO_H
#define __FIFO_H

#include "stm32f10x.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

#define FIFO_HIGH_DEPTH         16
#define FIFO_NORMAL_DEPTH       64
#define CAN_FRAME_DATA_LEN      8

/* CAN frame structure (matches CanRxMsg from STM32 StdPeriph) */
typedef struct {
    uint32_t StdId;
    uint8_t  IDE;
    uint8_t  RTR;
    uint8_t  DLC;
    uint8_t  Data[CAN_FRAME_DATA_LEN];
    uint8_t  FMI;
} CanRxFrame;

/* ---- HIGH priority FIFO (16 entries) ---- */
typedef struct {
    CanRxFrame         buffer[FIFO_HIGH_DEPTH];
    volatile uint8_t   head;
    volatile uint8_t   tail;
    volatile uint8_t   count;
    SemaphoreHandle_t  mutex;
} CAN_FIFO;

/* ---- NORMAL priority FIFO (64 entries) ---- */
typedef struct {
    CanRxFrame         buffer[FIFO_NORMAL_DEPTH];
    volatile uint8_t   head;
    volatile uint8_t   tail;
    volatile uint8_t   count;
    SemaphoreHandle_t  mutex;
} CAN_NormalFIFO;

/* ---- Extern globals ---- */
extern CAN_FIFO        g_fifo_high;
extern CAN_NormalFIFO  g_fifo_normal;
extern volatile uint16_t g_fifo_high_overflow_cnt;
extern volatile uint16_t g_fifo_normal_overflow_cnt;

/* ---- API ---- */
void     FIFO_Init(void);
uint8_t  FIFO_High_Push(CanRxFrame *frame);      /* 0=OK, 1=overflow */
uint8_t  FIFO_Normal_Push(CanRxFrame *frame);    /* 0=OK, 1=overflow */
uint8_t  FIFO_High_Pop(CanRxFrame *frame);       /* 0=OK, 1=empty */
uint8_t  FIFO_Normal_Pop(CanRxFrame *frame);     /* 0=OK, 1=empty */
uint8_t  FIFO_High_Count(void);
uint8_t  FIFO_Normal_Count(void);
uint8_t  FIFO_High_Full(void);
uint8_t  FIFO_Normal_Full(void);

#endif /* __FIFO_H */
