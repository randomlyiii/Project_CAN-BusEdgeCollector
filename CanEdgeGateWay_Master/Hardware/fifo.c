/**
 * Dual FIFO buffer implementation
 *
 * Lock ordering rule: always lock HIGH first, then NORMAL.
 * Never nest reverse-order locks to prevent deadlock.
 */

#include "fifo.h"
#include <string.h>

/* ---- FIFO instances ---- */
CAN_FIFO         g_fifo_high;
CAN_NormalFIFO   g_fifo_normal;
volatile uint16_t g_fifo_high_overflow_cnt   = 0;
volatile uint16_t g_fifo_normal_overflow_cnt = 0;

/* ---- Init ---- */
void FIFO_Init(void)
{
    memset(&g_fifo_high, 0, sizeof(g_fifo_high));
    memset(&g_fifo_normal, 0, sizeof(g_fifo_normal));

    g_fifo_high.mutex   = xSemaphoreCreateMutex();
    g_fifo_normal.mutex = xSemaphoreCreateMutex();

    g_fifo_high_overflow_cnt   = 0;
    g_fifo_normal_overflow_cnt = 0;
}

/* ==================== HIGH FIFO ==================== */

uint8_t FIFO_High_Push(CanRxFrame *frame)
{
    uint8_t overflow = 0;

    if (xSemaphoreTake(g_fifo_high.mutex, pdMS_TO_TICKS(5)) != pdPASS)
        return 1;  /* mutex timeout */

    if (g_fifo_high.count >= FIFO_HIGH_DEPTH) {
        /* Overflow: drop oldest frame (advance tail) */
        g_fifo_high.tail = (g_fifo_high.tail + 1) % FIFO_HIGH_DEPTH;
        g_fifo_high.count--;
        g_fifo_high_overflow_cnt++;
        overflow = 1;
    }

    memcpy(&g_fifo_high.buffer[g_fifo_high.head], frame, sizeof(CanRxFrame));
    g_fifo_high.head = (g_fifo_high.head + 1) % FIFO_HIGH_DEPTH;
    g_fifo_high.count++;

    xSemaphoreGive(g_fifo_high.mutex);
    return overflow;
}

uint8_t FIFO_High_Pop(CanRxFrame *frame)
{
    if (g_fifo_high.count == 0) return 1;

    if (xSemaphoreTake(g_fifo_high.mutex, pdMS_TO_TICKS(5)) != pdPASS)
        return 1;

    if (g_fifo_high.count == 0) {
        xSemaphoreGive(g_fifo_high.mutex);
        return 1;
    }

    memcpy(frame, &g_fifo_high.buffer[g_fifo_high.tail], sizeof(CanRxFrame));
    g_fifo_high.tail = (g_fifo_high.tail + 1) % FIFO_HIGH_DEPTH;
    g_fifo_high.count--;

    xSemaphoreGive(g_fifo_high.mutex);
    return 0;
}
uint8_t FIFO_High_Count(void) { return (uint8_t)g_fifo_high.count; }
uint8_t FIFO_High_Full(void)  { return (g_fifo_high.count >= FIFO_HIGH_DEPTH) ? 1 : 0; }

/* ==================== NORMAL FIFO ==================== */

uint8_t FIFO_Normal_Push(CanRxFrame *frame)
{
    uint8_t overflow = 0;

    if (xSemaphoreTake(g_fifo_normal.mutex, pdMS_TO_TICKS(5)) != pdPASS)
        return 1;

    if (g_fifo_normal.count >= FIFO_NORMAL_DEPTH) {
        /*
         * Overflow: prefer to drop a level-2 (low-freq) frame.
         * Scan backward from tail to find one with Data[0]==2.
         */
        uint8_t i, pos = g_fifo_normal.head;
        uint8_t found = 0;
        for (i = 0; i < FIFO_NORMAL_DEPTH; i++) {
            pos = (pos == 0) ? (FIFO_NORMAL_DEPTH - 1) : (pos - 1);
            if (g_fifo_normal.buffer[pos].Data[0] == 2) {
                /* Overwrite this level-2 entry */
                memcpy(&g_fifo_normal.buffer[pos], frame, sizeof(CanRxFrame));
                found = 1;
                break;
            }
        }
        if (!found) {
            /* All level-1 frames: drop oldest */
            g_fifo_normal.tail = (g_fifo_normal.tail + 1) % FIFO_NORMAL_DEPTH;
            g_fifo_normal.count--;
            memcpy(&g_fifo_normal.buffer[g_fifo_normal.head], frame, sizeof(CanRxFrame));
            g_fifo_normal.head = (g_fifo_normal.head + 1) % FIFO_NORMAL_DEPTH;
            g_fifo_normal.count++;
        }
        g_fifo_normal_overflow_cnt++;
        overflow = 1;
    } else {
        memcpy(&g_fifo_normal.buffer[g_fifo_normal.head], frame, sizeof(CanRxFrame));
        g_fifo_normal.head = (g_fifo_normal.head + 1) % FIFO_NORMAL_DEPTH;
        g_fifo_normal.count++;
    }

    xSemaphoreGive(g_fifo_normal.mutex);
    return overflow;
}

uint8_t FIFO_Normal_Pop(CanRxFrame *frame)
{
    if (g_fifo_normal.count == 0) return 1;

    if (xSemaphoreTake(g_fifo_normal.mutex, pdMS_TO_TICKS(5)) != pdPASS)
        return 1;

    if (g_fifo_normal.count == 0) {
        xSemaphoreGive(g_fifo_normal.mutex);
        return 1;
    }

    memcpy(frame, &g_fifo_normal.buffer[g_fifo_normal.tail], sizeof(CanRxFrame));
    g_fifo_normal.tail = (g_fifo_normal.tail + 1) % FIFO_NORMAL_DEPTH;
    g_fifo_normal.count--;

    xSemaphoreGive(g_fifo_normal.mutex);
    return 0;
}
uint8_t FIFO_Normal_Count(void) { return (uint8_t)g_fifo_normal.count; }
uint8_t FIFO_Normal_Full(void)  { return (g_fifo_normal.count >= FIFO_NORMAL_DEPTH) ? 1 : 0; }
