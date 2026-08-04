/**
 * STM32F10x Interrupt Service Routines — Phase 2 (FreeRTOS)
 *
 * NOTE: The following handlers are provided elsewhere:
 *   SysTick_Handler   → System/Delay.c (calls FreeRTOS xPortSysTickHandler)
 *   SVC_Handler       → FreeRTOS/src/port.c (vPortSVCHandler)
 *   PendSV_Handler    → FreeRTOS/src/port.c (xPortPendSVHandler)
 *   USB_LP_CAN1_RX0_IRQHandler → Hardware/CAN_User.c
 *   CAN1_SCE_IRQHandler → Hardware/CAN_User.c
 */

#include "stm32f10x_it.h"
#include "oled.h"

/* ---- Cortex-M3 exception handlers (non-FreeRTOS) ---- */

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    /* Try to show fault on OLED if initialized */
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
 * SysTick_Handler — Defined in System/Delay.c.
 * SVC_Handler     — Defined in FreeRTOS/src/port.c.
 * PendSV_Handler  — Defined in FreeRTOS/src/port.c.
 * CAN ISRs        — Defined in Hardware/CAN_User.c.
 */
