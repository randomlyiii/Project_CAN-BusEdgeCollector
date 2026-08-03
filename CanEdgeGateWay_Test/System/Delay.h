#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

/* 微秒延时: DWT 周期计数器 (不影响 SysTick) */
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

/* DWT 周期计数器初始化 */
void Delay_DWT_Init(void);

/* SysTick 1ms 时基 (裸机, 无 FreeRTOS) */
void     Delay_InitTick(void);
uint32_t Delay_GetTick(void);      /* ms */
uint32_t Delay_GetUs(void);        /* us, 高精度节奏控制用 */
void     SysTick_Handler(void);

#endif
