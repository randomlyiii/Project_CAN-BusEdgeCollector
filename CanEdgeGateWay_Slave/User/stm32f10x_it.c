/**
 * STM32F10x Interrupt Service Routines — Slave Phase 2 (FreeRTOS)
 *
 * NOTE: The following handlers are provided elsewhere:
 *   SysTick_Handler   → System/Delay.c (calls FreeRTOS xPortSysTickHandler)
 *   SVC_Handler       → FreeRTOS/src/port.c (vPortSVCHandler)
 *   PendSV_Handler    → FreeRTOS/src/port.c (xPortPendSVHandler)
 *   CAN1_RX0_IRQHandler → Hardware/CAN_User.c
 */

#include "stm32f10x_it.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1) { }
}

void MemManage_Handler(void)
{
    while (1) { }
}

void BusFault_Handler(void)
{
    while (1) { }
}

void UsageFault_Handler(void)
{
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
