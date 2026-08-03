/**
 * STM32F10x Interrupt Service Routines — CAN TX Stress (裸机)
 *
 * NOTE: The following handlers are provided elsewhere:
 *   SysTick_Handler  → System/Delay.c (1ms 时基)
 *   CAN ISRs         → 本工具无 (纯轮询 TX, 不开中断)
 */

#include "stm32f10x_it.h"
#include "oled.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "HARD FAULT!");
    OLED_ShowString(2, 1, "Check stack");
    while (1) { }
}

void MemManage_Handler(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "MEM MANAGE FLT");
    while (1) { }
}

void BusFault_Handler(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "BUS FAULT!");
    while (1) { }
}

void UsageFault_Handler(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "USAGE FAULT!");
    while (1) { }
}

void DebugMon_Handler(void)
{
}

/*
 * SysTick_Handler — Defined in System/Delay.c (1ms 时基).
 * CAN ISRs        — 本工具无, 纯轮询 TX。
 */
