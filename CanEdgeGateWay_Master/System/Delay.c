/**
 * Delay functions — FreeRTOS compatible
 *
 * FreeRTOS takes over SysTick for its 1ms tick.
 * Microsecond delays use the DWT cycle counter to avoid
 * interfering with the FreeRTOS scheduler.
 */

#include "stm32f10x.h"
#include "delay.h"
#include "../FreeRTOS/inc/FreeRTOS.h"

/* ---- DWT register definitions (CMSIS v1.30 compat) ---- */

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

/* ---- DWT microsecond delay (does not touch SysTick) ---- */

static uint8_t g_dwt_enabled = 0;

void Delay_DWT_Init(void)
{
    /* Enable DWT cycle counter (see 阶段二踩坑预判 #8) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_dwt_enabled = 1;
}

void Delay_us(uint32_t xus)
{
    if (!g_dwt_enabled) {
        /* Fallback: busy-wait with NOP */
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

/* ---- SysTick / FreeRTOS tick ---- */

void Delay_InitTick(void)
{
    Delay_DWT_Init();

    /* SysTick is configured by FreeRTOS vPortSetupTimerInterrupt().
       We just need to ensure the NVIC priority is set low (lowest urgency). */
    NVIC_SetPriority(SysTick_IRQn, 15);
}

/**
 * SysTick_Handler — re-mapped to FreeRTOS's xPortSysTickHandler
 * by the FreeRTOSConfig.h macro:
 *   #define xPortSysTickHandler  SysTick_Handler
 */
void SysTick_Handler(void)
{
    /* FreeRTOS tick increment + possible context switch */
    extern void xPortSysTickHandler(void);
#if (INCLUDE_xTaskGetSchedulerState == 1)
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
#else
    xPortSysTickHandler();
#endif
}

/**
 * Get millisecond tick count (FreeRTOS-based)
 * Returns actual milliseconds regardless of RTOS tick rate.
 */
uint32_t Delay_GetTick(void)
{
    return (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/**
 * Blocking delay using FreeRTOS (only when scheduler is running)
 */
void Delay_BlockMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}
