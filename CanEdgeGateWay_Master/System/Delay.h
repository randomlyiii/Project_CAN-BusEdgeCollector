#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

/* SysTick 1ms滴答定时器 */
void     Delay_InitTick(void);
uint32_t Delay_GetTick(void);
void     SysTick_Handler(void);
void     Delay_BlockMs(uint32_t ms);

#endif
