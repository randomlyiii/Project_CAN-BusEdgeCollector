#include "stm32f10x.h"

void Delay_us(uint32_t xus)
{
    SysTick->LOAD = 72 * xus;
    SysTick->VAL = 0x00;
    SysTick->CTRL = 0x00000005;
    while(!(SysTick->CTRL & 0x00010000));
    SysTick->CTRL = 0x00000004;
}

void Delay_ms(uint32_t xms)
{
    while(xms--)
    {
        Delay_us(1000);
    }
}

void Delay_s(uint32_t xs)
{
    while(xs--)
    {
        Delay_ms(1000);
    }
}

/* ========== SysTick 1ms 滴答计数器 ========== */

static volatile uint32_t g_sys_tick = 0;

void Delay_InitTick(void)
{
    if (SysTick_Config(SystemCoreClock / 1000))
        while (1);
    NVIC_SetPriority(SysTick_IRQn, 15);
}

void SysTick_Handler(void)
{
    g_sys_tick++;
}

uint32_t Delay_GetTick(void)
{
    uint32_t tick;
    __disable_irq();
    tick = g_sys_tick;
    __enable_irq();
    return tick;
}

void Delay_BlockMs(uint32_t ms)
{
    uint32_t start = Delay_GetTick();
    while (Delay_GetTick() - start < ms);
}
