/**
 * Delay functions — 裸机 (无 FreeRTOS)
 *
 * 微秒延时用 DWT 周期计数器 (72MHz 指令级精度)。
 * 毫秒时基用 SysTick 1ms 中断, 主循环读 g_sys_tick_ms。
 */

#include "stm32f10x.h"
#include "delay.h"

/* ---- DWT 寄存器定义 (CMSIS v1.30 兼容) ---- */

#ifndef DWT_BASE
#define DWT_BASE            (0xE0001000UL)
#endif

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
    volatile uint32_t CPICNT;
    volatile uint32_t EXCCNT;
    volatile uint32_t SLEEPCNT;
    volatile uint32_t LSUCNT;
    volatile uint32_t FOLDCNT;
    volatile uint32_t PCSR;
} DWT_Type;

#define DWT                 ((DWT_Type *)DWT_BASE)
#define DWT_CTRL_CYCCNTENA_Msk   (1UL << 0)

#ifndef CoreDebug_DEMCR_TRCENA_Msk
#define CoreDebug_DEMCR_TRCENA_Msk  (1UL << 24)
#endif

/* ---- 1ms SysTick 时基 ---- */

static volatile uint32_t g_sys_tick_ms = 0;

void SysTick_Handler(void)
{
    g_sys_tick_ms++;
}

/* ---- DWT 周期计数器 ---- */

static uint8_t g_dwt_enabled = 0;

void Delay_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_dwt_enabled = 1;
}

void Delay_us(uint32_t xus)
{
    if (!g_dwt_enabled) {
        uint32_t cycles = xus * 72;
        while (cycles--) __NOP();
        return;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = xus * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks) { }
}

void Delay_ms(uint32_t xms)
{
    while (xms--) Delay_us(1000);
}

void Delay_s(uint32_t xs)
{
    while (xs--) Delay_ms(1000);
}

/* ---- SysTick 1ms 时基初始化 (裸机) ---- */

void Delay_InitTick(void)
{
    Delay_DWT_Init();
    SysTick_Config(SystemCoreClock / 1000);   /* 1ms 中断 */
    NVIC_SetPriority(SysTick_IRQn, 15);
}

uint32_t Delay_GetTick(void)
{
    return g_sys_tick_ms;
}

uint32_t Delay_GetUs(void)
{
    if (!g_dwt_enabled) return 0;
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000));
}
