/**
 * Delay functions — FreeRTOS compatible
 *
 * FreeRTOS takes over SysTick for its 1ms tick.
 * Microsecond delays use the DWT cycle counter to avoid
 * interfering with the FreeRTOS scheduler.
 */

#include "stm32f10x.h"
#include "delay.h"

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
    xPortSysTickHandler();
}

/**
 * Get millisecond tick count (FreeRTOS-based)
 */
uint32_t Delay_GetTick(void)
{
    return (uint32_t)xTaskGetTickCount();
}

/**
 * Blocking delay using FreeRTOS (only when scheduler is running)
 */
void Delay_BlockMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}
