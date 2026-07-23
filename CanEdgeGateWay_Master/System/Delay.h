#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

/* SysTick 1ms 滴答定时器 (用于非阻塞延时和超时检测) */
void     Delay_InitTick(void);             // 初始化 SysTick, 1ms 中断
uint32_t Delay_GetTick(void);              // 获取当前系统 tick (ms)
void     SysTick_Handler(void);            // SysTick 中断处理
void     Delay_BlockMs(uint32_t ms);       // 基于 tick 的阻塞延时

#endif
