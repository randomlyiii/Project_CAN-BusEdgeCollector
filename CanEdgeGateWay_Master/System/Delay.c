#include "stm32f10x.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
    SysTick->LOAD = 72 * xus;				//设置定时器重装值
    SysTick->VAL = 0x00;					//清空当前计数值
    SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
    while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
    SysTick->CTRL = 0x00000004;				//关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
    while(xms--)
    {
        Delay_us(1000);
    }
}

/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
    while(xs--)
    {
        Delay_ms(1000);
    }
}

/* ========== SysTick 1ms 滴答计数器 (用于非阻塞延时和超时检测) ========== */

static volatile uint32_t g_sys_tick = 0;

/**
  * @brief  初始化 SysTick 为 1ms 中断模式
  * @note   需要在 main 开始时调用一次。SystemCoreClock 由 system_stm32f10x.c 提供
  *         该定时器启动后 SysTick_Handler 每 1ms 被调用一次
  * @retval 无
  */
void Delay_InitTick(void)
{
    if (SysTick_Config(SystemCoreClock / 1000))
        while (1);						// 配置失败时死循环(通常不会发生)
    NVIC_SetPriority(SysTick_IRQn, 15);	// 最低优先级
}

/**
  * @brief  SysTick 中断处理函数 (1ms 自增)
  * @note   由启动文件 startup_stm32f10x_md.s 中的 SysTick_Handler 调用
  * @retval 无
  */
void SysTick_Handler(void)
{
    g_sys_tick++;
}

/**
  * @brief  获取当前系统滴答计数 (ms)
  * @retval 当前 tick 值 (上电后毫秒数)
  */
uint32_t Delay_GetTick(void)
{
    uint32_t tick;

    __disable_irq();
    tick = g_sys_tick;
    __enable_irq();

    return tick;
}

/**
  * @brief  基于 tick 的阻塞延时
  * @param  ms 要延时的毫秒数
  * @retval 无
  */
void Delay_BlockMs(uint32_t ms)
{
    uint32_t start = Delay_GetTick();

    while (Delay_GetTick() - start < ms);
}
