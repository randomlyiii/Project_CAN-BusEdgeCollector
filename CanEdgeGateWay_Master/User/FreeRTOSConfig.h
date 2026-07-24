/**
 * FreeRTOS V10.4.6 Configuration for CAN Edge Gateway Master
 * STM32F103C8T6 @ 72MHz, Keil MDK-ARM
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f10x.h"

/*---------------------------------------------------------------------------*/
#define configCPU_CLOCK_HZ                      (72000000UL)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    (8)
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)
#define configTOTAL_HEAP_SIZE                   ((size_t)(15 * 1024))
#define configMAX_TASK_NAME_LEN                 (16)
#define configUSE_16_BIT_TICKS                  0
#define configUSE_MUTEXES                       1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_TASK_STACK_DEPTH            (128)
#define configTIMER_QUEUE_LENGTH                (10)
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   3
#define configQUEUE_REGISTRY_SIZE               8

#define configKERNEL_INTERRUPT_PRIORITY         (15 << 4)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (5 << 4)

/* Cortex-M3: 4 priority bits, highest prio=0, lowest=15 (shifted by 4) */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY          0x0F
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     0x05

#define configPRIO_BITS                  __NVIC_PRIO_BITS
#define configKERNEL_YIELD_PRIORITY      (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_PRIORITIES_MASK        ((1UL << (uint32_t)configMAX_PRIORITIES) - 1)

#define configASSERT(x)                  if ((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;); }

/* FreeRTOS heap hooks */
#define vPortSVCHandler        SVC_Handler
#define xPortPendSVHandler     PendSV_Handler
#define xPortSysTickHandler    SysTick_Handler

/* Optional features */
#define INCLUDE_vTaskPrioritySet              1
#define INCLUDE_uxTaskPriorityGet             1
#define INCLUDE_vTaskDelete                   1
#define INCLUDE_vTaskCleanUpResources         0
#define INCLUDE_vTaskSuspend                  1
#define INCLUDE_vTaskDelayUntil               1
#define INCLUDE_vTaskDelay                    1
#define INCLUDE_eTaskGetState                 1
#define INCLUDE_xTimerPendFunctionCall        1
#define INCLUDE_xTaskGetSchedulerState        1
#define INCLUDE_uxTaskGetStackHighWaterMark   1
#define INCLUDE_xTaskGetIdleTaskHandle        1
#define INCLUDE_pcTaskGetTaskName             1
#define INCLUDE_xTaskAbortDelay               0

#endif /* FREERTOS_CONFIG_H */
