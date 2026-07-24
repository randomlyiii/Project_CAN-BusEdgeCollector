#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

/* Microsecond delay using DWT cycle counter (FreeRTOS safe) */
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

/* DWT cycle counter init */
void Delay_DWT_Init(void);

/* SysTick / FreeRTOS 1ms tick */
void     Delay_InitTick(void);
uint32_t Delay_GetTick(void);
void     SysTick_Handler(void);
void     Delay_BlockMs(uint32_t ms);

#endif
