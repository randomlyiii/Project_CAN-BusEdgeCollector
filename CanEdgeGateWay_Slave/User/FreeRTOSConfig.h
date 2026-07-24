/**
 * FreeRTOS V10.4.6 Configuration for CAN Edge Gateway Slave
 * STM32F103C8T6 @ 72MHz, Keil MDK-ARM
 *
 * ==================== FreeRTOS 内核路径 ====================
 * 当前: 共用项目根目录下的 FreeRTOS/ 内核
 *
 * 如需本工程独立使用不同版本/裁剪的内核:
 *   1. 将 FreeRTOS/ 复制到本工程目录下 (如 CanEdgeGateWay_Slave/FreeRTOS/)
 *   2. 将文件末尾 #include 路径改为 "../FreeRTOS/inc/FreeRTOS.h"
 *   3. Keil 工程 → Options → C/C++ → Include Paths 同步修改
 *
 * 文件组织:
 *   独立模式:             #include "../FreeRTOS/inc/FreeRTOS.h"
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f10x.h"

/*---------------------------------------------------------------------------*/
#define configCPU_CLOCK_HZ                      (72000000UL)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    (8)
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)
#define configTOTAL_HEAP_SIZE                   ((size_t)(10 * 1024))
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

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY          0x0F
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     0x05

#define configPRIO_BITS                  __NVIC_PRIO_BITS
#define configKERNEL_YIELD_PRIORITY      (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_PRIORITIES_MASK        ((1UL << (uint32_t)configMAX_PRIORITIES) - 1)

#define configASSERT(x)                  if ((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;); }

/* ISR handler name mapping (macros rename FreeRTOS default names → vector table names) */
#define vPortSVCHandler        SVC_Handler
#define xPortPendSVHandler     PendSV_Handler
/* SysTick: handled by System/Delay.c → calls xPortSysTickHandler() */

#define configASSERT_DEFINED   0   /* skip portASSERT_IF_INTERRUPT_PRIORITY_INVALID checks */

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

/* ---- 引入 FreeRTOS 内核 (必须在所有 config 宏定义之后) ---- */
#include "../FreeRTOS/inc/FreeRTOS.h"

#endif /* FREERTOS_CONFIG_H */
