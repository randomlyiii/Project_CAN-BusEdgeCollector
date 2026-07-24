/**
 * FreeRTOS V10.4.6 Configuration for CAN Edge Gateway Master
 * STM32F103C8T6 (Cortex-M3) @ 72MHz, Keil MDK-ARM
 *
 * ==================== 中断优先级速查 ====================
 * NVIC 分组:     PriorityGroup_4 (4 位全抢占, 无子优先级)
 * NVIC 有效位:   4 位 → 优先级 0-15 (0=最高, 15=最低)
 * NVIC 寄存器值: 优先级 << 4 (如 priority 5 → 0x50)
 *
 * PendSV / SysTick:   最低优先级 15 (0xF0)
 * 可用 FromISR API:   优先级 5-15 (0x50-0xF0)
 * 不可用 FromISR API: 优先级 0-4  (0x00-0x40, 仅限极速 ISR)
 *
 * 关键规则: 所有调用 xSemaphoreGiveFromISR 等 FromISR 函数
 * 的 ISR 优先级必须在数值上 >= configMAX_SYSCALL_INTERRUPT_PRIORITY
 * =========================================================
 *
 * 文件组织:
 *   共用内核:             #include "../FreeRTOS/inc/FreeRTOS.h"
 *   独立内核:             将 FreeRTOS/ 复制到本工程目录下
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f10x.h"

/*---------------------------------------------------------------------------*/
/*------------------------ 基础时钟与调度配置 -------------------------------*/

#define configCPU_CLOCK_HZ                      (72000000UL)
#define configTICK_RATE_HZ                      (1000)              /* 1ms per tick */
#define configUSE_PREEMPTION                    1                   /* 抢占式调度     */
#define configUSE_TIME_SLICING                  1                   /* 同优先级轮转   */
#define configIDLE_SHOULD_YIELD                 1                   /* Idle 主动让出  */

/*---------------------------------------------------------------------------*/
/*------------------------ 任务优先级与栈 -----------------------------------*/

#define configMAX_PRIORITIES                    (8)                 /* 0-7, 数值越大优先级越高 */
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)     /* Idle 任务栈 (word)      */
#define configMAX_TASK_NAME_LEN                 (16)
#define configTOTAL_HEAP_SIZE                   ((size_t)(12 * 1024))   /* 12KB 堆 */
#define configUSE_16_BIT_TICKS                  0                   /* 32-bit tick */

/*---------------------------------------------------------------------------*/
/*------------------------ 内核对象开关 -------------------------------------*/

#define configUSE_MUTEXES                       1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   3

#define configQUEUE_REGISTRY_SIZE               8                   /* 调试用队列注册表 */
#define configUSE_APPLICATION_TASK_TAG          0

/*---------------------------------------------------------------------------*/
/*------------------------ 软件定时器 ---------------------------------------*/

#define configUSE_TIMERS                        0  /* 调试: 暂时关闭软件定时器 */
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)  /* 优先级 7 (最高) */
#define configTIMER_TASK_STACK_DEPTH            (128)
#define configTIMER_QUEUE_LENGTH                (10)

/*---------------------------------------------------------------------------*/
/*------------------------ 钩子函数 -----------------------------------------*/

#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2  /* 方法2: 魔数填充 + SP 检查 */
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/*---------------------------------------------------------------------------*/
/*------------------------ 内存分配策略 -------------------------------------*/

#define configSUPPORT_DYNAMIC_ALLOCATION        1                   /* xTaskCreate 等动态 API */
#define configSUPPORT_STATIC_ALLOCATION         0                   /* 本项目不用静态分配     */

/*---------------------------------------------------------------------------*/
/*--------------------- 中断优先级 (Cortex-M3 BASEPRI 机制) -----------------*/
/*
 *  FreeRTOS 使用 BASEPRI 寄存器实现临界区, BASEPRI 接受的是 NVIC 8-bit
 *  优先级寄存器的原始值 (即优先级已左移 4 位后的值)。
 *
 *  configKERNEL_INTERRUPT_PRIORITY:
 *    设置 PendSV (#14) 和 SysTick (#15) 的中断优先级。
 *    必须是系统最低优先级。STM32F103 4 位优先级: 15 << 4 = 0xF0
 *
 *  configMAX_SYSCALL_INTERRUPT_PRIORITY:
 *    FreeRTOS 临界区的 BASEPRI 阈值。优先级高于此值 (数值更小) 的 ISR
 *    不可调用任何 FreeRTOS API。优先级等于或低于此值的 ISR 可安全调用
 *    xSemaphoreGiveFromISR 等 FromISR 函数。
 *    5 << 4 = 0x50 → ISR 优先级 5-15 可调 API, 0-4 不可调。
 */

/*
 *  configPRIO_BITS = 4, 所以:
 *  configPRIO_BITS = 4, 所以:
 *   - configKERNEL_INTERRUPT_PRIORITY     = 15 << 4 = 0xF0 (最低优先级)
 *   - configMAX_SYSCALL_INTERRUPT_PRIORITY =  5 << 4 = 0x50 (API 阈值)
 *
 *  注意: 这两个宏在 port.c 内联汇编中直接引用 (mov r0, #宏值),
 *  必须是纯数值常量, 不能带 UL 后缀或嵌套表达式, 否则 ARMCC 汇编器报错.
 */
#define configPRIO_BITS                         4

#define configKERNEL_INTERRUPT_PRIORITY         0xF0    /* 15 << 4, 最低优先级 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    0x50    /*  5 << 4, API 阈值  */

/* 方便人阅读的未移位值 (非标准, 本工程约定, 内核不使用) */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY          15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     5

/*---------------------------------------------------------------------------*/
/*------------------------ 断言与中断校验 -----------------------------------*/
/*
 *  configASSERT_DEFINED = 1 → 开启:
 *    1. xPortStartScheduler() 中校验 NVIC 优先级位数与实际硬件一致
 *    2. 运行时 portASSERT_IF_INTERRUPT_PRIORITY_INVALID() 检查 FromISR
 *       调用的 ISR 优先级是否合法
 *  开发/调试阶段必须开启。量产确认无问题后可设为 0 缩减代码体积。
 */

#define configASSERT_DEFINED                    0  /* 调试期间关闭, 确认调度器启动后再开 */

#define configASSERT(x)                         \
    if ((x) == 0) {                              \
        portDISABLE_INTERRUPTS();                \
        for (;;);                                \
    }

/*---------------------------------------------------------------------------*/
/*-------------------- ISR 函数名映射 (ARMCC/Keil) ---------------------------*/
/*
 * 启动文件 startup_stm32f10x_md.s 中弱定义了 SVC_Handler、PendSV_Handler、
 * SysTick_Handler（均为 B . 死循环）。
 *
 * SVC / PendSV: port.c 源码已直接命名为 SVC_Handler 和 PendSV_Handler
 *              （非宏映射），编译后为强符号，覆盖启动文件的弱定义。
 *
 * SysTick:     System/Delay.c 提供 SysTick_Handler 强符号，内部调用
 *              xPortSysTickHandler()。SysTick 硬件由 port.c 的
 *              vPortSetupTimerInterrupt() 配置和启动。
 *
 * 以下宏保留用于兼容（内核源码内部如有引用 vPortSVCHandler 则生效）：
 */
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
/* SysTick_Handler → 见 System/Delay.c */

/*---------------------------------------------------------------------------*/
/*------------------------ 可选 API 函数开关 --------------------------------*/

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_pcTaskGetTaskName               1
#define INCLUDE_xTaskAbortDelay                 0

/*---------------------------------------------------------------------------*/
/* 引入 FreeRTOS 内核 (必须在所有 config 宏定义之后)                          */
#include "../FreeRTOS/inc/FreeRTOS.h"

#endif /* FREERTOS_CONFIG_H */
